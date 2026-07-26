#!/usr/bin/env python3
"""
RFS STT node.

Records and transcribes microphone input, and forwards the user's spoken
interruption to the family members. Recording, VAD, Gemini transcription, config
loading, and vote counting logic is handled by `stt_backend` (no rclpy
dependency); this file only handles ROS2 publish/subscribe (the communication
layer) and starting/managing the recording loop that runs in the background.

Beginner's guide — one user interruption, step by step (see `_recorder_loop`):
  1. The microphone is always listening. Once the user finishes speaking,
     `record_and_transcribe` (in `stt_backend`) turns it into text.
  2. This node publishes that text on `rfs_user_intervention` so every family
     member can vote on who should answer (`vote_callback` collects the votes).
  3. `_count_votes` (using `stt_backend.count_votes`) picks the winner, and this
     node publishes the decision so only that member responds.
  4. It waits for `rfs_stt_resume` before listening for the next interruption.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import time
import os
os.environ["NO_GCE_CHECK"] = "true"
import json
import asyncio
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from rfs_stt.stt_backend import (
    GeminiLiveRecorder,
    load_stt_config,
    load_family_config,
    select_audio_device,
    count_votes,
)

HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")
HISTORY_FILE = os.path.join(DB_DIR, "conversation_history.txt")


class RFSSTT(Node):
    """ROS2 node that recognizes microphone speech, decides the responder by vote, and notifies the family."""

    def __init__(self):
        super().__init__('rfs_stt')

        # --- ROS2 communication setup ---
        qos_pl = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.intervention_pub = self.create_publisher(String, 'rfs_user_intervention', qos_pl)
        self.speech_status_pub = self.create_publisher(Bool, 'rfs_speech_status', 10)
        self.ready_event = threading.Event()
        self.resume_event = threading.Event()
        self.create_subscription(String, 'rfs_stt_resume', self.resume_callback, 10)
        self.initial_scenario_sub = self.create_subscription(String, 'rfs_initial_scenario_generated', self.initial_scenario_callback, qos_pl)

        # Vote collection
        self.create_subscription(String, 'rfs_responder_vote', self.vote_callback, 10)
        self._votes = {}          # {role: voted_role}
        self._vote_lock = threading.Lock()
        self._vote_event = threading.Event()
        self._expected_voters = 0

        # --- Config loading (logic layer) ---
        self.stt_config = load_stt_config()
        self.language = self.stt_config["language"]
        self.family_config = load_family_config()
        self.get_logger().info(f"STT Language Mode: {self.language}")
        self.get_logger().info(f"VAD Config: {self.stt_config}")
        self.get_logger().info(f"Family Config: {self.family_config}")

        # Device selection happens at startup and can be re-triggered
        self._selected_device = None
        self._device_reselect_event = threading.Event()
        self.recorder = None  # Will be created after device selection
        self.api_key = os.environ.get("GEMINI_API_KEY")
        self.recorder_thread = threading.Thread(target=self._recorder_loop, daemon=True)
        self.recorder_thread.start()

    def initial_scenario_callback(self, msg: String):
        """Wait for the first conversation generation to complete before allowing the recording loop to start."""
        if msg.data == "completed":
            time.sleep(5)
            self.ready_event.set()
            self.destroy_subscription(self.initial_scenario_sub)

    def resume_callback(self, msg: String):
        self.resume_event.set()

    def _on_speech_start(self):
        self.intervention_pub.publish(String(data='user_speech_started'))

    def _on_speech_end(self):
        self.intervention_pub.publish(String(data='user_speech_ended'))

    def _on_speech_status_change(self, is_active: bool):
        self.speech_status_pub.publish(Bool(data=is_active))

    def vote_callback(self, msg: String):
        """Collect responder votes from each family member."""
        try:
            data = json.loads(msg.data)
            voter = data.get("voter", "").lower()
            voted_for = data.get("voted_for", "").lower()
            self.get_logger().info(f"Vote received: {voter} -> {voted_for}")
            with self._vote_lock:
                self._votes[voter] = voted_for
                if len(self._votes) >= self._expected_voters:
                    self._vote_event.set()
        except Exception as e:
            self.get_logger().error(f"Error parsing vote: {e}")

    def _count_votes(self) -> str:
        """Count the collected votes and return the winner (the counting logic lives in the logic layer)."""
        with self._vote_lock:
            votes = dict(self._votes)
        return count_votes(votes, self.family_config, logger=self.get_logger())

    def _create_recorder(self, device_index):
        """Create/recreate the recorder (logic layer) for the given device."""
        self.recorder = GeminiLiveRecorder(
            on_start=self._on_speech_start,
            on_end=self._on_speech_end,
            on_speech_status_change=self._on_speech_status_change,
            logger=self.get_logger(),
            language=self.language,
            vad_aggressiveness=self.stt_config["vad_aggressiveness"],
            silence_duration_s=self.stt_config["silence_duration_s"],
            speech_trigger_frames=self.stt_config["speech_trigger_frames"],
            vad_debug=self.stt_config.get("vad_debug", False),
            vad_energy_threshold=self.stt_config.get("vad_energy_threshold", 0.0),
            device_index=device_index,
        )

    def _recorder_loop(self):
        """Run the device-selection -> record -> vote -> decide-responder loop on a background thread."""
        # Step 1: Select device before waiting for scenario
        self._selected_device = select_audio_device()
        self._create_recorder(self._selected_device)

        # Step 2: Wait for initial scenario
        self.ready_event.wait()

        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)

        # Start a stdin listener thread for device re-selection
        self._reselect_requested = False
        def stdin_listener():
            """Listens for the Enter key to request a device re-selection."""
            while rclpy.ok():
                try:
                    input()  # Block until Enter is pressed
                    self._reselect_requested = True
                    self.get_logger().info("Device re-selection requested. Will apply after current recording.")
                except EOFError:
                    break
        stdin_thread = threading.Thread(target=stdin_listener, daemon=True)
        stdin_thread.start()

        async def run():
            while rclpy.ok():
                # Check if re-selection was requested
                if self._reselect_requested:
                    self._reselect_requested = False
                    self._selected_device = select_audio_device()
                    self._create_recorder(self._selected_device)

                transcript = await self.recorder.record_and_transcribe()
                if transcript.strip():
                    print(f"\n[Recognized] User: {transcript.strip()}\n")

                    # Step 1: Publish transcript to all members for voting
                    self.get_logger().info(f"Broadcasting transcript for voting: {transcript.strip()}")
                    with self._vote_lock:
                        self._votes.clear()
                        self._expected_voters = len(self.family_config)
                    self._vote_event.clear()

                    vote_request = {
                        "text": transcript.strip()
                    }
                    self.intervention_pub.publish(
                        String(data=f"user_speech_transcribed:{json.dumps(vote_request)}")
                    )

                    # Step 2: Wait for votes (timeout 10s)
                    self._vote_event.wait(timeout=10.0)

                    # Step 3: Count votes
                    selected_member = self._count_votes()
                    self.get_logger().info(f"Vote result: {selected_member} selected as responder")

                    # Step 4: Publish decision
                    decision_payload = {
                        "responder": selected_member,
                        "text": transcript.strip()
                    }
                    self.intervention_pub.publish(String(data=f"user_decision:{json.dumps(decision_payload)}"))
                    self.resume_event.clear()
                    self.resume_event.wait()
        _loop.run_until_complete(run())


def main():
    rclpy.init()
    node = RFSSTT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
