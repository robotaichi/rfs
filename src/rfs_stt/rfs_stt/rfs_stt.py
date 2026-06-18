#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import time
import sounddevice as sd
import os
os.environ["NO_GCE_CHECK"] = "true"
import json
import numpy as np
from google import genai
from google.genai import types
from collections import deque
import asyncio
import webrtcvad
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import io

HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")
HISTORY_FILE = os.path.join(DB_DIR, "conversation_history.txt")

class GeminiLiveRecorder:
    def __init__(
        self,
        model: str = "gemini-3.1-flash-lite",
        api_key_env: str = "GEMINI_API_KEY",
        vad_aggressiveness: int = 3,
        silence_duration_s: float = 2.0,
        speech_trigger_frames: int = 5,
        on_start: callable = lambda: None,
        on_end: callable = lambda: None,
        on_speech_status_change: callable = lambda x: None,
        logger=None,
        language: str = "en",
        vad_debug: bool = False,
        vad_energy_threshold: float = 0.0,
        device_index: int = None,
    ):
        self.api_key = os.environ.get(api_key_env)
        if not self.api_key:
            if logger: logger.error("GEMINI_API_KEY environment variable is not set.")
        
        self.model = model
        self.logger = logger
        self.on_start = on_start
        self.on_end = on_end
        self.on_speech_status_change = on_speech_status_change
        self.vad_debug = vad_debug
        self.vad_energy_threshold = vad_energy_threshold
        self.device_index = device_index
        
        self.lang_code = "ja-JP" if language == "ja" else "en-US"
        self.language = language
        
        self.sample_rate = 16000
        self.vad = webrtcvad.Vad(vad_aggressiveness)
        self.frame_duration_ms = 30
        self.frame_size = int(self.sample_rate * (self.frame_duration_ms / 1000.0))
        self.max_silence_frames = int(silence_duration_s * 1000 / self.frame_duration_ms)
        self.speech_trigger_frames = speech_trigger_frames
        self.on_start = on_start
        self.on_end = on_end
        self.on_speech_status_change = on_speech_status_change
        self._is_speech_active = False

    async def _record_audio(self) -> bytes:
        """Record audio using VAD to detect speech start/end. Returns raw PCM bytes at 16kHz."""
        speech_started = False
        silence_counter = 0
        speech_frame_counter = 0
        pre_buffer = deque(maxlen=self.speech_trigger_frames + 5)
        recorded_frames = []

        # Determine the actual sample rate to use
        target_rate = self.sample_rate  # 16000
        actual_rate = target_rate
        needs_resample = False

        if self.device_index is not None:
            dev_info = sd.query_devices(self.device_index)
            dev_default_rate = int(dev_info['default_samplerate'])
            # Try target rate first; if device doesn't list it, use its default
            # Common rates that work with webrtcvad: 8000, 16000, 32000, 48000
            try:
                sd.check_input_settings(device=self.device_index, samplerate=target_rate, channels=1, dtype='int16')
            except Exception:
                self.logger.info(f"Device {self.device_index} does not support {target_rate}Hz. Using {dev_default_rate}Hz with resampling.")
                actual_rate = dev_default_rate
                needs_resample = True

        actual_frame_size = int(actual_rate * (self.frame_duration_ms / 1000.0))

        try:
            with sd.RawInputStream(
                samplerate=actual_rate,
                blocksize=actual_frame_size,
                dtype="int16",
                channels=1,
                device=self.device_index,
            ) as mic:
                self.logger.info(f"RFS STT: Idle (Waiting for speech...) [Device rate: {actual_rate}Hz]")
                while rclpy.ok():
                    frame, _ = mic.read(actual_frame_size)
                    
                    # Resample to 16kHz if needed (for VAD and transcription)
                    if needs_resample:
                        audio_float = np.frombuffer(frame, dtype=np.int16).astype(np.float32)
                        # Simple linear interpolation resampling
                        target_len = int(len(audio_float) * target_rate / actual_rate)
                        indices = np.linspace(0, len(audio_float) - 1, target_len)
                        resampled = np.interp(indices, np.arange(len(audio_float)), audio_float)
                        frame_16k = resampled.astype(np.int16).tobytes()
                    else:
                        frame_16k = bytes(frame)
                    
                    # Energy calculation (on 16kHz data)
                    audio_data = np.frombuffer(frame_16k, dtype=np.int16)
                    energy = np.sqrt(np.mean(audio_data.astype(np.float32)**2))
                    
                    # VAD expects exactly frame_size samples at 16kHz
                    # Ensure frame is exact size for VAD
                    vad_frame = frame_16k[:self.frame_size * 2]  # 2 bytes per int16 sample
                    if len(vad_frame) < self.frame_size * 2:
                        vad_frame = vad_frame + b'\x00' * (self.frame_size * 2 - len(vad_frame))
                    
                    is_speech_vad = self.vad.is_speech(vad_frame, self.sample_rate)
                    is_speech = is_speech_vad and (energy >= self.vad_energy_threshold)

                    if self.vad_debug:
                        status_char = "S" if is_speech else "."
                        if not is_speech and is_speech_vad: status_char = "x"
                        self.logger.info(f"VAD: {status_char} | E: {energy:6.1f} | S:{speech_frame_counter:2d} | Z:{silence_counter:2d}", once=False)

                    if is_speech and not self._is_speech_active:
                        self._is_speech_active = True
                        self.on_speech_status_change(True)
                    elif not is_speech and self._is_speech_active:
                        self._is_speech_active = False
                        self.on_speech_status_change(False)

                    if not speech_started:
                        pre_buffer.append(frame_16k)
                        if is_speech:
                            speech_frame_counter += 1
                            if speech_frame_counter >= self.speech_trigger_frames:
                                self.logger.info("RFS STT: Recording...")
                                self.on_start()
                                speech_started = True
                                recorded_frames.extend(list(pre_buffer))
                                pre_buffer.clear()
                        else:
                            speech_frame_counter = 0
                    else:
                        recorded_frames.append(frame_16k)
                        if is_speech: silence_counter = 0
                        else: silence_counter += 1
                        
                        if silence_counter > self.max_silence_frames:
                            self.logger.info("RFS STT: Speech ended, processing...")
                            self.on_end()
                            break
        except Exception as e:
            self.logger.error(f"Error in STT recorder: {e}")
        
        return b"".join(recorded_frames)

    def _transcribe_with_rest(self, audio_pcm: bytes) -> str:
        """Send recorded audio to Gemini REST API for transcription."""
        import requests
        import base64
        import wave
        import tempfile
        import io
        import socket
        import urllib3.util.connection as urllib3_cn
        urllib3_cn.allowed_gai_family = lambda: socket.AF_INET
        
        if not audio_pcm:
            return ""
        
        # Convert raw PCM to WAV format in memory
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_pcm)
        wav_data = wav_buffer.getvalue()
        
        # Encode as base64
        audio_b64 = base64.b64encode(wav_data).decode('utf-8')
        
        lang_instruction = "日本語で" if self.language == "ja" else "in English"
        
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent?key={self.api_key}"
        headers = {"Content-Type": "application/json"}
        payload = {
            "contents": [{
                "parts": [
                    {"text": f"Transcribe the following audio {lang_instruction}. Output ONLY the transcription text, nothing else."},
                    {"inlineData": {"mimeType": "audio/wav", "data": audio_b64}}
                ]
            }]
        }
        
        try:
            self.logger.info("RFS STT: Sending audio to Gemini REST API for transcription...")
            res = requests.post(url, headers=headers, json=payload, timeout=30.0)
            if res.status_code == 200:
                transcript = res.json()["candidates"][0]["content"]["parts"][0]["text"].strip()
                self.logger.info(f"RFS STT: Transcription result: {transcript}")
                return transcript
            else:
                self.logger.error(f"RFS STT: API returned {res.status_code}: {res.text[:200]}")
                return ""
        except Exception as e:
            self.logger.error(f"RFS STT: Transcription error: {e}")
            return ""

    async def record_and_transcribe(self) -> str:
        audio_data = await self._record_audio()
        if not audio_data:
            return ""
        # Run REST API call in a thread to avoid blocking the event loop
        transcript = await asyncio.to_thread(self._transcribe_with_rest, audio_data)
        return transcript

class RFSSTT(Node):
    def __init__(self):
        super().__init__('rfs_stt')
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

        self.stt_config = self._load_stt_config()
        self.language = self.stt_config["language"]
        self.family_config = self._load_family_config()
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

    def _load_stt_config(self):
        config_data = {
            "language": "en",
            "vad_aggressiveness": 3,
            "silence_duration_s": 2.0,
            "speech_trigger_frames": 5,
            "vad_debug": False,
            "vad_energy_threshold": 0.0
        }
        try:
            # RFS style path resolution
            home = os.path.expanduser("~")
            paths = [
                os.path.join(home, "rfs/src/rfs_config/config/config.json"),
                os.path.join(home, "rfs/install/rfs_config/share/rfs_config/config/config.json"),
            ]
            
            config_file = next((p for p in paths if os.path.exists(p)), None)
            
            if config_file:
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    config_data["language"] = config.get("language", "en").lower()
                    config_data["vad_aggressiveness"] = config.get("vad_aggressiveness", 3)
                    config_data["silence_duration_s"] = config.get("silence_duration_s", 2.0)
                    config_data["speech_trigger_frames"] = config.get("speech_trigger_frames", 5)
                    config_data["vad_debug"] = config.get("vad_debug", False)
                    config_data["vad_energy_threshold"] = config.get("vad_energy_threshold", 0.0)
        except: pass
        return config_data

    def _load_family_config(self):
        try:
            home = os.path.expanduser("~")
            paths = [
                os.path.join(home, "rfs/src/rfs_config/config/config.json"),
                os.path.join(home, "rfs/install/rfs_config/share/rfs_config/config/config.json"),
            ]
            config_file = next((p for p in paths if os.path.exists(p)), None)
            if config_file:
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    return [r.lower() for r in config.get("family_config", ["father", "mother", "daughter"])]
        except: pass
        return ["father", "mother", "daughter"]

    def initial_scenario_callback(self, msg: String):
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
        """Collect votes from family members."""
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

    def _tally_votes(self) -> str:
        """Count votes and return the winner (majority). Tie-break: first in family_config order."""
        with self._vote_lock:
            votes = dict(self._votes)
        
        if not votes:
            return self.family_config[0] if self.family_config else "father"
        
        # Count votes
        counts = {}
        for voted_for in votes.values():
            counts[voted_for] = counts.get(voted_for, 0) + 1
        
        self.get_logger().info(f"Vote tally: {counts}")
        
        max_count = max(counts.values())
        # Tie-break by family_config order (first listed wins)
        for member in self.family_config:
            if counts.get(member, 0) == max_count:
                return member
        
        # Fallback
        return max(counts, key=counts.get)

    def _select_audio_device(self):
        """Interactive audio device selection. Returns selected device index."""
        devices = sd.query_devices()
        input_devices = []
        for i, d in enumerate(devices):
            if d['max_input_channels'] > 0:
                input_devices.append((i, d))
        
        if not input_devices:
            print("\n[RFS STT] No input devices found! Using system default.")
            return None
        
        print("\n" + "=" * 60)
        print("  RFS STT - マイクデバイス選択 / Microphone Selection")
        print("=" * 60)
        for idx, (dev_id, d) in enumerate(input_devices):
            marker = " *" if d == sd.query_devices(kind='input') else "  "
            print(f"  [{idx}]{marker} {d['name']}")
            print(f"        (channels: {d['max_input_channels']}, rate: {d['default_samplerate']:.0f}Hz)")
        print("=" * 60)
        print("  * = current default device")
        print("  番号を入力してEnter / Enter number and press Enter:")
        
        while True:
            try:
                choice = input("  > ").strip()
                if choice == "":
                    # Use default
                    print(f"  Using default device.")
                    return None
                idx = int(choice)
                if 0 <= idx < len(input_devices):
                    dev_id, d = input_devices[idx]
                    print(f"  Selected: [{idx}] {d['name']}")
                    print("=" * 60 + "\n")
                    return dev_id
                else:
                    print(f"  Invalid number. Enter 0-{len(input_devices)-1}")
            except ValueError:
                print(f"  Enter a number (0-{len(input_devices)-1}) or press Enter for default.")
            except EOFError:
                return None

    def _create_recorder(self, device_index):
        """Create or re-create the recorder with the given device index."""
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
        # Step 1: Select device before waiting for scenario
        self._selected_device = self._select_audio_device()
        self._create_recorder(self._selected_device)
        
        # Step 2: Wait for initial scenario
        self.ready_event.wait()
        
        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)
        
        # Start a stdin listener thread for device re-selection
        self._reselect_requested = False
        def stdin_listener():
            """Listen for Enter key press to trigger device re-selection."""
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
                    self._selected_device = self._select_audio_device()
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
                    
                    # Step 3: Tally votes
                    selected_member = self._tally_votes()
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
