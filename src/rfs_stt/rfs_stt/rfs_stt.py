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
        """Record audio using VAD to detect speech start/end. Returns raw PCM bytes."""
        speech_started = False
        silence_counter = 0
        speech_frame_counter = 0
        pre_buffer = deque(maxlen=self.speech_trigger_frames + 5)
        recorded_frames = []

        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=self.frame_size,
                dtype="int16",
                channels=1,
            ) as mic:
                self.logger.info("RFS STT: Idle (Waiting for speech...)")
                while rclpy.ok():
                    frame, _ = mic.read(self.frame_size)
                    
                    # Energy calculation
                    audio_data = np.frombuffer(frame, dtype=np.int16)
                    energy = np.sqrt(np.mean(audio_data.astype(np.float32)**2))
                    
                    is_speech_vad = self.vad.is_speech(frame, self.sample_rate)
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
                        pre_buffer.append(bytes(frame))
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
                        recorded_frames.append(bytes(frame))
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

        self.stt_config = self._load_stt_config()
        self.language = self.stt_config["language"]
        self.get_logger().info(f"STT Language Mode: {self.language}")
        self.get_logger().info(f"VAD Config: {self.stt_config}")

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
            vad_energy_threshold=self.stt_config.get("vad_energy_threshold", 0.0)
        )
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

    def _determine_responder_with_gemini(self, transcript: str) -> str:
        try:
            history = ""
            if os.path.exists(HISTORY_FILE):
                with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                    lines = f.readlines()
                filtered = [l for l in lines if not (l.startswith("[THERAPIST_") or l.startswith("[SYSTEM_UPDATE"))]
                history = "".join(filtered)
            
            home = os.path.expanduser("~")
            paths = [
                os.path.join(home, "rfs/src/rfs_config/config/config.json"),
                os.path.join(home, "rfs/install/rfs_config/share/rfs_config/config/config.json"),
            ]
            config_file = next((p for p in paths if os.path.exists(p)), None)
            
            family_config = []
            if config_file:
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    family_config = config.get("family_config", [])
            
            if not family_config:
                family_config = ["father", "mother", "daughter"]
            
            prompt = f"""
You are a coordinator for a family robot conversation simulation.
Based on the following conversation history and the user's speech, determine which family member should respond to the user.

Available family members: {family_config}

Conversation History:
{history}

User Speech:
"{transcript}"

Which family member is the most appropriate to respond? Respond with ONLY the name of the family member from the list above in lowercase (e.g., father, mother, daughter, son). Do not include any other words or punctuation.
"""
            import requests
            url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-3.1-flash-lite:generateContent?key={self.api_key}"
            headers = {"Content-Type": "application/json"}
            payload = {
                "contents": [{"parts": [{"text": prompt}]}],
                "generationConfig": {
                    "temperature": 0.0
                }
            }
            res = requests.post(url, headers=headers, json=payload, timeout=15.0)
            if res.status_code == 200:
                ans = res.json()["candidates"][0]["content"]["parts"][0]["text"].strip().lower()
            else:
                raise RuntimeError(f"Gemini API returned {res.status_code}: {res.text}")
            for m in family_config:
                if m.lower() in ans:
                    return m.lower()
            return family_config[0].lower()
        except Exception as e:
            self.get_logger().error(f"Error determining responder with Gemini: {e}")
            return "father"

    def _recorder_loop(self):
        self.ready_event.wait()
        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)
        async def run():
            while rclpy.ok():
                transcript = await self.recorder.record_and_transcribe()
                if transcript.strip():
                    print(f"\n[Recognized] User: {transcript.strip()}\n")
                    selected_member = self._determine_responder_with_gemini(transcript.strip())
                    self.get_logger().info(f"Selected responder: {selected_member}")
                    
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
