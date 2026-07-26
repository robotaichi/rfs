#!/usr/bin/env python3
"""
RFS TTS node.

Receives speech requests (service calls) from the family members, synthesizes
speech using the Gemini API, and plays it back on the speaker assigned to that
role (linked to a toio) or the default output. Speech synthesis, playback, and
audio output destination control are handled by `tts_backend` (no rclpy
dependency); this file only handles ROS2 publish/subscribe/service (the
communication layer) and manages the playback queue.

Beginner's guide — one request, step by step:
  1. A family member calls the `rfs_speak_text` service; `speak_text_callback`
     parses the request and starts synthesizing the audio in the background
     right away (`_queue_audio_task`), then puts it in `self.playback_queue`.
  2. `_playback_worker` runs forever in the background, taking one queued
     item at a time: it picks the right speaker (`sink`), waits for that
     item's audio to finish synthesizing, plays it, and publishes
     `rfs_tts_status`/`rfs_tts_finished` so the family member knows it's done.
  3. `interrupt_tts_callback` can stop everything early (a user started
     talking), mute all speakers, or resume — see the `rfs_interrupt_tts` topic.
"""

import rclpy
import time
import os
os.environ["NO_GCE_CHECK"] = "true"
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rfs_interfaces.srv import TTSService
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import asyncio
import json
import csv
import io
from typing import Optional
from ament_index_python.packages import get_package_share_directory

from rfs_tts.tts_backend import GeminiTTS, AudioSinkController

# Constants
try:
    SAVE_DIR = os.path.join(get_package_share_directory('rfs_config'), 'config')
except Exception:
    SAVE_DIR = os.path.expanduser("~/rfs/src/rfs_config/config")

CONFIG_FILE = os.path.join(SAVE_DIR, 'config.json')
HISTORY_FILE = os.path.join(SAVE_DIR, 'conversation_history.txt')
SINGLE_MEMBER_ROLE = 'androgynous_communication_robot'


class RFSTTS(Node):
    """ROS2 node (communication layer) that synthesizes and plays back speech in response to requests."""

    def __init__(self, loop):
        super().__init__('rfs_tts')
        self.loop = loop
        self.playback_queue = asyncio.Queue()
        self._current_playback_task = None
        self.current_speaker_role = None
        self.connected_toios: dict[str, str] = {}
        self.muted_sinks_original_volumes = {}
        self.initial_sink_volumes = {}
        self.hdmi_sink = "alsa_output.pci-0000_01_00.1.hdmi-stereo-extra1"
        self.use_hdmi_fallback = False

        # --- ROS2 communication setup ---
        self.srv = self.create_service(TTSService, 'rfs_speak_text', self.speak_text_callback)
        self.tts_status_pub = self.create_publisher(String, 'rfs_tts_status', 10)
        self.tts_finished_pub = self.create_publisher(String, 'rfs_tts_finished', 10)
        self.fallback_finished_pub = self.create_publisher(String, 'rfs_fallback_finished', 10)

        self.create_subscription(String, 'rfs_interrupt_tts', self.interrupt_tts_callback, 10, callback_group=ReentrantCallbackGroup())
        self.create_subscription(String, 'rfs_intervention_resolved', self.intervention_resolved_callback, 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.initialization_pub = self.create_publisher(String, 'rfs_tts_initialization', qos_profile)
        self.create_subscription(String, 'rfs_toio_status', self.toio_status_callback, qos_profile)

        self.initialization_pub.publish(String(data="tts_initialized"))
        self.get_logger().info("RFS TTS Started.")

        # --- Logic layer (no rclpy dependency: speech synthesis, output destination control) ---
        self.client = GeminiTTS(self.get_logger(), self.loop)
        self.sinks = AudioSinkController(self.get_logger())

        # Defer config loading and volume detection to the event loop
        self.loop.create_task(self._async_init())
        self.loop.create_task(self._playback_worker())

    async def _async_init(self):
        """Async initialization for config and pulse sinks."""
        await self.load_config()
        await self._get_initial_sink_volumes()
        self.get_logger().info("TTS Node Async Initialization Complete.")

    async def _get_initial_sink_volumes(self):
        available_sinks = await self.sinks.get_available_sinks()
        for sink in available_sinks:
            volume = await self.sinks.get_raw_sink_volume(sink)
            if volume is not None:
                self.initial_sink_volumes[sink] = volume

    def toio_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            status = data.get("status", "").strip().lower()
            if status == "toios_ready":
                positions = data.get("positions", {})
                self.connected_toios.clear()
                for role, pos_data in positions.items():
                    if role in self.role_map:
                        self.connected_toios[role] = self.role_map[role]
            elif status == "initializing":
                self.connected_toios.clear()
        except Exception as e:
            self.get_logger().error(f"Error in toio_status_callback: {e}")

    def intervention_resolved_callback(self, msg: String):
        while not self.playback_queue.empty():
            try:
                self.playback_queue.get_nowait()
                self.playback_queue.task_done()
            except asyncio.QueueEmpty:
                break

    async def load_config(self):
        """Load the role<->speaker mapping and output mode from the config file, and decide the output destination policy."""
        self.role_map = {}
        self.speaker_map = {}
        self.family_roles = []
        self.chat_mode = 0
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                config_data = json.load(f)

            self.chat_mode = config_data.get('chat_mode', 0)
            self.family_roles = [role.lower() for role in config_data.get('family_config', [])]
            speaker_match = config_data.get('toio_speaker_match', [])

            for item in speaker_match:
                role_lower = item['role'].lower()
                self.role_map[role_lower] = item['speaker_id']
                if 'voicevox_speaker_id' in item:
                    self.speaker_map[role_lower] = item['voicevox_speaker_id']

            available_sinks = await self.sinks.get_available_sinks()
            # In Docker/Virtual environments, we prefer the default sink (None)
            # for the audio bridge over specific ghost HDMI devices.
            found_hdmi = self.sinks.find_hdmi_sink(available_sinks)
            if found_hdmi:
                self.get_logger().info(f"Found HDMI sink: {found_hdmi}")
                self.hdmi_sink = found_hdmi

            # If we are in Docker (auto_null exists), or no HDMI found, default is better
            if "auto_null" in available_sinks or not found_hdmi:
                self.get_logger().info("Prioritizing default sink for audio bridge.")
                self.hdmi_sink = None  # None means use PulseAudio default

            if self.chat_mode != 1:
                all_missing = True
                for sink in self.role_map.values():
                    if sink in available_sinks:
                        all_missing = False
                if all_missing and self.role_map:
                    self.use_hdmi_fallback = True

        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    async def _playback_worker(self):
        """Background worker that handles the playback queue one item at a time (wait for synthesis -> play -> tell others it's done)."""
        while rclpy.ok():
            try:
                role, text, voice_id, is_leader_response, delay, gen_task = await self.playback_queue.get()
                self.current_speaker_role = role

                if delay > 0:
                    await asyncio.sleep(delay)

                if is_leader_response:
                    await self._restore_volume()

                sink = self.hdmi_sink if (self.chat_mode == 1 or self.use_hdmi_fallback) else self.role_map.get(role)

                # If the configured sink is not a Bluetooth speaker, route to the default output destination
                if sink and "bluez" not in sink.lower():
                    self.get_logger().info(f"Sink '{sink}' for role '{role}' is not a Bluetooth speaker. Routing to default output destination.")
                    sink = None

                # Validate sink existence and fallback to default if missing
                available_sinks = await self.sinks.get_available_sinks()
                if sink and sink not in available_sinks:
                    self.get_logger().warn(f"Configured sink '{sink}' not found. Falling back to default sink.")
                    sink = None

                # In normal mode (not HDMI/chat mode), skip playback if this role has no sink mapping at all
                if sink is None and not (self.chat_mode == 1 or self.use_hdmi_fallback) and role not in self.role_map:
                    self.get_logger().warn(f"No sink mapping for role '{role}', skipping playback.")
                    if not gen_task.done(): gen_task.cancel()
                    continue

                # Wait for synthesis to complete (already running task)
                audio_file = None
                try:
                    self.get_logger().info(f"Playback worker: Waiting for synthesis task for {role}...")
                    audio_file = await asyncio.wait_for(gen_task, timeout=70.0)
                    self.get_logger().info(f"Playback worker: synthesis RESUMED/READY for {role}.")
                except asyncio.TimeoutError:
                    self.get_logger().error(f"Playback worker: synthesis TIMEOUT for {role}. Abandoning.")
                except Exception as e:
                    self.get_logger().error(f"Playback worker: synthesis FAILED for {role}: {e}")

                if audio_file and os.path.exists(audio_file):
                    self.get_logger().info(f"Playback worker: synthesis complete for {role}. Starting playback on {sink}...")
                    text_for_publish = text.replace(',', ';')
                    is_muted_by_intervention_str = "true" if self.muted_sinks_original_volumes else "false"
                    self.tts_status_pub.publish(String(data=f"start,{role},{text_for_publish},{is_muted_by_intervention_str}"))

                    self._current_playback_task = asyncio.create_task(self.client.play_audio(audio_file, sink))
                    await self._current_playback_task
                    self.get_logger().info(f"Playback worker: playback finished for {role}.")
                    os.remove(audio_file)

                    self.tts_status_pub.publish(String(data=f"end,{role}"))
                    self.tts_finished_pub.publish(String(data=f"finished,{role}"))
                else:
                    self.get_logger().error(f"Synthesis failed or audio file missing for {role}. Skipping playback.")
                    # Ensure status machine knows we hit the 'end' point of the attempt
                    self.tts_status_pub.publish(String(data=f"end,{role}"))
                    # Still publish finished so the state machine doesn't hang
                    self.tts_finished_pub.publish(String(data=f"finished,{role}"))
            except Exception as e:
                self.get_logger().error(f"Error in playback worker: {e}")
            finally:
                self.current_speaker_role = None
                self.playback_queue.task_done()

    async def _restore_volume(self):
        volumes_to_restore = self.muted_sinks_original_volumes or self.initial_sink_volumes
        for sink, original_volume in volumes_to_restore.items():
            await self.sinks.set_sink_volume(sink, original_volume)
        self.muted_sinks_original_volumes.clear()

    async def _queue_audio_task(self, role, text, voice_id, is_leader_response, delay):
        """Helper to create task and put it in queue within the event loop's thread."""
        try:
            gen_task = self.loop.create_task(self.client.generate_audio(text, voice_id))
            await self.playback_queue.put((role, text, voice_id, is_leader_response, delay, gen_task))
            self.get_logger().info(f"Task for {role} successfully queued.")
        except Exception as e:
            self.get_logger().error(f"Failed to queue audio task for {role}: {e}")

    def speak_text_callback(self, request, response):
        """`rfs_speak_text` service: parse the CSV line of the speech request and enqueue a synthesis task."""
        try:
            text_with_marker = request.text.strip()
            is_leader_response = False
            if text_with_marker.startswith("[LEADER_RESPONSE]"):
                is_leader_response = True
                text_with_marker = text_with_marker[len("[LEADER_RESPONSE]"):]

            reader = csv.reader(io.StringIO(text_with_marker), skipinitialspace=True)
            parts = next(reader)
            role = parts[0].lower()
            text_to_speak = parts[3]
            voice_id = parts[4].strip() if len(parts) > 4 else self.speaker_map.get(role, 'Kore')
            delay_val = request.delay

            self.get_logger().info(f"Srv: Scheduling synthesis for {role}...")
            # Use explicit argument binding in the lambda to avoid closure issues
            self.loop.call_soon_threadsafe(
                lambda r=role, t=text_to_speak, v=voice_id, l=is_leader_response, d=delay_val:
                asyncio.run_coroutine_threadsafe(self._queue_audio_task(r, t, v, l, d), self.loop)
            )
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in speak_text_callback: {e}")
            response.success = False
        return response

    async def _mute_sinks(self, sinks):
        for sink in sinks:
            try:
                v = await self.sinks.get_raw_sink_volume(sink)
                if v is not None: self.muted_sinks_original_volumes[sink] = v
                await self.sinks.set_sink_volume(sink, 0)
            except Exception: pass

    def _drain_playback_queue(self):
        """Remove every pending item from the playback queue, canceling its synthesis task if it's still running."""
        while not self.playback_queue.empty():
            try:
                item = self.playback_queue.get_nowait()
                if len(item) > 5 and hasattr(item[5], 'cancel'):
                    item[5].cancel()
                self.playback_queue.task_done()
            except asyncio.QueueEmpty: break

    def interrupt_tts_callback(self, msg: String):
        """On receiving `rfs_interrupt_tts`, stop playback, mute everything, or resume, depending on the message."""
        if msg.data == "stop_all":
            if not self.muted_sinks_original_volumes:
                sinks_to_mute = set([self.hdmi_sink]) if self.chat_mode == 1 else set(self.role_map.values())
                self.loop.create_task(self._mute_sinks(sinks_to_mute))

            self._drain_playback_queue()
        elif msg.data == "resume_all":
            self.loop.create_task(self._restore_volume())
        else:
            if self._current_playback_task and not self._current_playback_task.done():
                self._current_playback_task.cancel()
            self._drain_playback_queue()

    def destroy_node(self):
        """Explicitly cleanup audio processes on node shutdown."""
        self.get_logger().info("Shutting down TTS node and cleaning up audio processes...")
        if hasattr(self, 'client'):
            self.client.stop()
        super().destroy_node()

    async def _restore_volumes_on_exit(self):
        for sink, v in self.muted_sinks_original_volumes.items():
            await self.sinks.set_sink_volume(sink, v)


def main():
    rclpy.init()
    # Create the loop in the main thread of this node's worker
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    node = RFSTTS(loop)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the asyncio loop in a dedicated background thread
    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        # Restore original volumes on exit (fire and forget)
        if hasattr(node, '_restore_volumes_on_exit'):
            loop.create_task(node._restore_volumes_on_exit())

        loop.call_soon_threadsafe(loop.stop)
        t.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
