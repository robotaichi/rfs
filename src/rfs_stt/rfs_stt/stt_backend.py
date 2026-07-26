#!/usr/bin/env python3
"""
Logic layer for speech recognition (STT).

Handles microphone recording, VAD (voice activity detection), transcription using the
Gemini API, config loading, and vote counting. Does not depend on ROS2 communication
(topic publish/subscribe). Called from the rfs_stt node.

`GeminiLiveRecorder._record_audio` uses `rclpy.ok()` as its loop condition. This is
only used to stop the recording loop when the node shuts down, not for any ROS2
communication (publishers/subscribers) as such, so it is kept as-is here (changing
this stop condition could change shutdown behavior).
"""

import rclpy
import threading
import time
import sounddevice as sd
import os
import json
import numpy as np
from collections import deque
import asyncio
import webrtcvad


class GeminiLiveRecorder:
    """Logic class that detects speech with VAD, records it, and transcribes it using the Gemini REST API."""

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
        """Record audio while detecting speech start/end with VAD. Returns raw 16kHz PCM data (bytes)."""
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
        """Convert the recorded PCM audio to WAV and send it to the Gemini REST API for transcription."""
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
        """Record audio, then turn it into text, one step after the other."""
        audio_data = await self._record_audio()
        if not audio_data:
            return ""
        # Run REST API call in a thread to avoid blocking the event loop
        transcript = await asyncio.to_thread(self._transcribe_with_rest, audio_data)
        return transcript


def _find_config_file():
    """Find `config.json` by checking RFS's standard locations in order, returning the first one that exists."""
    home = os.path.expanduser("~")
    paths = [
        os.path.join(home, "rfs/src/rfs_config/config/config.json"),
        os.path.join(home, "rfs/install/rfs_config/share/rfs_config/config/config.json"),
    ]
    return next((p for p in paths if os.path.exists(p)), None)


def load_stt_config() -> dict:
    """Load VAD/language-related STT settings from `config.json` (following RFS's standard path resolution rules)."""
    config_data = {
        "language": "en",
        "vad_aggressiveness": 3,
        "silence_duration_s": 2.0,
        "speech_trigger_frames": 5,
        "vad_debug": False,
        "vad_energy_threshold": 0.0
    }
    try:
        config_file = _find_config_file()

        if config_file:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
                config_data["language"] = config.get("language", "en").lower()
                config_data["vad_aggressiveness"] = config.get("vad_aggressiveness", 3)
                config_data["silence_duration_s"] = config.get("silence_duration_s", 2.0)
                config_data["speech_trigger_frames"] = config.get("speech_trigger_frames", 5)
                config_data["vad_debug"] = config.get("vad_debug", False)
                config_data["vad_energy_threshold"] = config.get("vad_energy_threshold", 0.0)
    except Exception:
        pass
    return config_data


def load_family_config() -> list:
    """Load the family composition (list of role names) from `config.json`."""
    try:
        config_file = _find_config_file()
        if config_file:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
                return [r.lower() for r in config.get("family_config", ["father", "mother", "daughter"])]
    except Exception:
        pass
    return ["father", "mother", "daughter"]


def select_audio_device():
    """Let the user interactively choose a microphone device, and return the selected device ID (None = default)."""
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


def count_votes(votes: dict, family_config: list, logger=None) -> str:
    """Count the votes (role -> voted-for) and return the winner with the most votes (ties broken by family_config order)."""
    if not votes:
        return family_config[0] if family_config else "father"

    counts = {}
    for voted_for in votes.values():
        counts[voted_for] = counts.get(voted_for, 0) + 1

    if logger:
        logger.info(f"Vote count: {counts}")

    max_count = max(counts.values())
    # Tie-break by family_config order (first listed wins)
    for member in family_config:
        if counts.get(member, 0) == max_count:
            return member

    # Fallback
    return max(counts, key=counts.get)
