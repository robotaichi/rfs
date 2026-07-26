#!/usr/bin/env python3
"""
Logic layer for speech synthesis (TTS) and audio output destination control.

Handles speech synthesis/playback using the Gemini API, and sink (output
destination) detection/volume control through PipeWire/PulseAudio. Does not
depend on ROS2 communication (topics/services). Called from the rfs_tts node.
"""

import os
import asyncio
import tempfile
import subprocess
import re
import json
import wave
from typing import Optional


class GeminiTTS:
    """Logic class that synthesizes speech using the Gemini API and plays it back using ffplay/pw-play/aplay."""

    def __init__(self, logger, loop):
        self.logger = logger
        self.loop = loop
        self.api_key = os.environ.get("GEMINI_API_KEY")
        if not self.api_key:
            self.logger.error("GEMINI_API_KEY environment variable is not set.")
            raise RuntimeError("GEMINI_API_KEY is missing")
        self.model_id = "gemini-2.5-flash-preview-tts"
        self._current_playback_process = None

    async def generate_audio(self, text: str, voice: str) -> Optional[str]:
        """Synthesize speech for the given text/voice and return the temp WAV file path (None on failure)."""
        try:
            # Valid Gemini voices from voice_list.txt
            valid_voices = [
                "Zephyr", "Puck", "Charon", "Kore", "Fenrir", "Leda", "Orus", "Aoede", "Callirrhoe",
                "Autonoe", "Enceladus", "Iapetus", "Umbriel", "Algieba", "Despina", "Erinome",
                "Algenib", "Rasalgethi", "Laomedeia", "Achernar", "Alnilam", "Schedar", "Gacrux",
                "Pulcherrima", "Achird", "Zubenelgenubi", "Vindemiatrix", "Sadachbia", "Sadaltager", "Sulafat"
            ]
            if voice not in valid_voices:
                self.logger.warn(f"Invalid voice '{voice}' requested. Defaulting to 'Kore'.")
                voice = "Kore"

            def _get_config():
                return types.GenerateContentConfig(
                    response_modalities=["AUDIO"],
                    speech_config=types.SpeechConfig(
                        voice_config=types.VoiceConfig(
                            prebuilt_voice_config=types.PrebuiltVoiceConfig(
                                voice_name=voice,
                            )
                        )
                    ),
                )

            # Parallel synthesis enabled (no semaphore wrap)
            audio_data = None
            last_error = None

            self.logger.info(f"Generating audio for voice '{voice}' via {self.model_id} (REST Parallel)...")
            for attempt in range(5):
                try:
                    self.logger.info(f"API attempt {attempt+1} starting for {voice}...")
                    import requests
                    import base64
                    import socket
                    import urllib3.util.connection as urllib3_cn
                    urllib3_cn.allowed_gai_family = lambda: socket.AF_INET

                    def call_api():
                        url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_id}:generateContent?key={self.api_key}"
                        headers = {"Content-Type": "application/json"}
                        payload = {
                            "contents": [{"parts": [{"text": text}]}],
                            "generationConfig": {
                                "responseModalities": ["AUDIO"],
                                "speechConfig": {
                                    "voiceConfig": {
                                        "prebuiltVoiceConfig": {
                                            "voiceName": voice
                                        }
                                    }
                                }
                            }
                        }
                        return requests.post(url, headers=headers, json=payload, timeout=30.0)

                    res = await asyncio.to_thread(call_api)
                    if res.status_code == 200:
                        res_data = res.json()
                        b64_data = res_data["candidates"][0]["content"]["parts"][0]["inlineData"]["data"]
                        audio_data = base64.b64decode(b64_data)
                        self.logger.info(f"API attempt {attempt+1} SUCCESS for {voice}.")
                        break
                    else:
                        raise RuntimeError(f"REST API status {res.status_code}: {res.text}")
                except Exception as e:
                    last_error = e
                    self.logger.warn(f"Gemini TTS attempt {attempt+1} failed for {voice}: {e}")

                # Sleep a bit before retry unless it's the last attempt
                if attempt < 4:
                    await asyncio.sleep(1.0)

            if not audio_data:
                self.logger.error(f"Failed to generate audio for {voice} after all attempts: {last_error}")
                return None

            # The SDK returns binary data for the audio content
            self.logger.info(f"Extracting audio data for {voice}...")

            # Create a unique temporary file
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav", dir=tempfile.gettempdir()) as f:
                temp_filename = f.name

            self.logger.info(f"Writing WAV file to {temp_filename}...")
            # Gemini TTS returns PCM data (24kHz, mono, 16-bit)
            with wave.open(temp_filename, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(24000)
                wf.writeframes(audio_data)

            self.logger.info(f"Audio pre-generated successfully: {temp_filename}")
            return temp_filename
        except Exception as e:
            self.logger.error(f"Fatal error in generate_audio for {voice}: {e}")
            return None

    async def play_audio(self, filename: str, sink: str = None):
        """Play a generated WAV file using whichever playback command is available (ffplay/pw-play/aplay)."""
        try:
            import shutil
            play_command = None
            env = os.environ.copy()

            # 1. Check for ffplay
            if shutil.which("ffplay"):
                play_command = ["ffplay", "-nodisp", "-autoexit", filename]
                if sink:
                    env["PULSE_SINK"] = sink
            # 2. Check for pw-play (PipeWire)
            elif shutil.which("pw-play"):
                if sink:
                    play_command = ["pw-play", f"--target={sink}", filename]
                else:
                    play_command = ["pw-play", filename]
            # 3. Check for aplay (ALSA)
            elif shutil.which("aplay"):
                play_command = ["aplay", filename]
                if sink:
                    env["PULSE_SINK"] = sink
            else:
                self.logger.error("No suitable audio playback utility (ffplay, pw-play, aplay) found.")
                return

            self._current_playback_process = await asyncio.create_subprocess_exec(
                *play_command, env=env, stdout=subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE
            )
            _, stderr = await self._current_playback_process.communicate()
            if self._current_playback_process.returncode != 0:
                self.logger.error(f"Playback failed ({self._current_playback_process.returncode}): {stderr.decode().strip() if stderr else 'No stderr'}")
        except asyncio.CancelledError:
            self.stop()
            raise
        except Exception as e:
            self.logger.warn(f"Unexpected error during audio playback: {e}")
        finally:
            self._current_playback_process = None

    def stop(self):
        if self._current_playback_process and self._current_playback_process.returncode is None:
            try:
                self._current_playback_process.terminate()
            except ProcessLookupError:
                pass
            self._current_playback_process = None


class AudioSinkController:
    """Detects and controls the volume of audio output destinations (sinks) using PipeWire (wpctl) / PulseAudio (pactl)."""

    def __init__(self, logger):
        self.logger = logger

    def find_hdmi_sink(self, available_sinks: list) -> Optional[str]:
        """Find the HDMI output among the available sinks."""
        for sink in available_sinks:
            if "hdmi" in sink.lower():
                return sink
        return None

    async def _run_pw_dump(self) -> Optional[list]:
        """Run `pw-dump` and return its parsed JSON list, or None if it's unavailable or fails."""
        try:
            proc = await asyncio.create_subprocess_exec(
                "pw-dump",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=2.0)
            if proc.returncode == 0:
                return json.loads(stdout.decode())
        except Exception:
            pass
        return None

    async def get_wp_node_id(self, sink_name: str) -> Optional[int]:
        """Get the PipeWire node ID for the given sink name."""
        data = await self._run_pw_dump()
        if data:
            for obj in data:
                if obj.get('type') == 'PipeWire:Interface:Node':
                    props = obj.get('info', {}).get('props', {})
                    if props.get('node.name') == sink_name:
                        return obj.get('id')
        return None

    async def get_available_sinks(self) -> list:
        """Get the list of available output sinks from PipeWire (pw-dump) or PulseAudio (pactl)."""
        sinks = []
        # 1. Try pw-dump (PipeWire)
        data = await self._run_pw_dump()
        if data:
            for obj in data:
                if obj.get('type') == 'PipeWire:Interface:Node':
                    props = obj.get('info', {}).get('props', {})
                    if props.get('media.class') == 'Audio/Sink':
                        name = props.get('node.name')
                        if name:
                            sinks.append(name)
            if sinks:
                return sinks

        # 2. Try pactl (PulseAudio)
        try:
            proc = await asyncio.create_subprocess_exec(
                "pactl", "list", "sinks", "short",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env={"LANG": "C"}
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=2.0)
            if proc.returncode == 0:
                for line in stdout.splitlines():
                    parts = line.decode().split('\t')
                    if len(parts) > 1:
                        sinks.append(parts[1])
                if sinks:
                    return sinks
        except Exception:
            pass

        return sinks

    async def set_sink_volume(self, sink: str, volume: float):
        """Set the volume of the given sink (tries PipeWire first, falls back to PulseAudio on failure)."""
        # 1. Try PipeWire (wpctl)
        node_id = await self.get_wp_node_id(sink)
        if node_id is not None:
            try:
                wp_vol = volume
                if wp_vol > 1.0:
                    wp_vol = 1.0  # Cap/normalize for PipeWire
                proc = await asyncio.create_subprocess_exec(
                    "wpctl", "set-volume", str(node_id), f"{wp_vol:.2f}"
                )
                await asyncio.wait_for(proc.wait(), timeout=2.0)
                return
            except Exception:
                pass

        # 2. Try PulseAudio (pactl)
        try:
            pactl_vol = volume
            if pactl_vol <= 1.0:
                pactl_vol = int(pactl_vol * 65536)
            else:
                pactl_vol = int(pactl_vol)

            proc = await asyncio.create_subprocess_exec(
                "pactl", "set-sink-volume", sink, str(pactl_vol),
                env={"LANG": "C"}
            )
            await asyncio.wait_for(proc.wait(), timeout=2.0)
        except Exception as e:
            self.logger.warn(f"Failed to set volume for {sink}: {e}")

    async def get_raw_sink_volume(self, sink: str) -> Optional[float]:
        """Get the current volume of the given sink (tries PipeWire first, falls back to PulseAudio on failure)."""
        # 1. Try PipeWire (wpctl)
        node_id = await self.get_wp_node_id(sink)
        if node_id is not None:
            try:
                proc = await asyncio.create_subprocess_exec(
                    "wpctl", "get-volume", str(node_id),
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE
                )
                stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=2.0)
                if proc.returncode == 0:
                    match = re.search(r"Volume:\s*([\d\.]+)", stdout.decode())
                    if match:
                        return float(match.group(1))
            except Exception:
                pass

        # 2. Try PulseAudio (pactl)
        try:
            proc = await asyncio.create_subprocess_exec(
                "pactl", "list", "sinks",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env={"LANG": "C"}
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=2.0)
            if proc.returncode == 0:
                m = re.search(rf"Name: {re.escape(sink)}[\s\S]*?Volume:.*?(front-left|mono): (\d+) /", stdout.decode())
                if m:
                    return float(m.group(2))
        except Exception as e:
            self.logger.warn(f"Failed to get volume for {sink}: {e}")
            return None
