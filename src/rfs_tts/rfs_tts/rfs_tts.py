#!/usr/bin/env python3
import rclpy
import time
import os
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rfs_interfaces.srv import TTSService
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import random
import asyncio
import tempfile
import json
import subprocess
import re
import csv
import io
import google.genai as genai
from google.genai import types
import wave
from typing import Optional
from ament_index_python.packages import get_package_share_directory

# Constants
try:
    SAVE_DIR = os.path.join(get_package_share_directory('rfs_config'), 'config')
except Exception:
    SAVE_DIR = os.path.expanduser("~/rfs/src/rfs_config/config")

CONFIG_FILE = os.path.join(SAVE_DIR, 'config.json')
HISTORY_FILE = os.path.join(SAVE_DIR, 'conversation_history.txt')
SINGLE_MEMBER_ROLE = 'androgynous_communication_robot'

class GeminiTTS:
    def __init__(self, logger):
        self.logger = logger
        self.api_key = os.environ.get("GEMINI_API_KEY")
        if not self.api_key:
            self.logger.error("GEMINI_API_KEY environment variable is not set.")
            raise RuntimeError("GEMINI_API_KEY is missing")
        self.client = genai.Client(api_key=self.api_key)
        self.model_id = "gemini-2.5-flash-preview-tts"
        self._current_playback_process = None
        self._synthesis_semaphore = asyncio.Semaphore(1)

    async def generate_audio(self, text: str, voice: str) -> Optional[str]:
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

            def _api_call():
                return self.client.models.generate_content(
                    model=self.model_id,
                    contents=text,
                    config=types.GenerateContentConfig(
                        system_instruction="You are a dedicated text-to-speech engine. Your sole task is to take the provided text and convert it into audio exactly as written. Do not generate any text response, explanation, or commentary. Output ONLY audio.",
                        response_modalities=["AUDIO"],
                        speech_config=types.SpeechConfig(
                            voice_config=types.VoiceConfig(
                                prebuilt_voice_config=types.PrebuiltVoiceConfig(
                                    voice_name=voice,
                                )
                            )
                        ),
                    )
                )

            async with self._synthesis_semaphore:
                self.logger.info(f"Generating audio for voice '{voice}'...")
                # Run blocking API call in an executor with a more generous timeout
                response = await asyncio.wait_for(
                    asyncio.get_event_loop().run_in_executor(None, _api_call),
                    timeout=60.0
                )
            
            # The SDK returns binary data for the audio content
            audio_data = response.candidates[0].content.parts[0].inline_data.data
            
            # Create a unique temporary file for this specific audio task
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav", dir=tempfile.gettempdir()) as f:
                temp_filename = f.name
                
            # Gemini TTS returns PCM data (24kHz, mono, 16-bit)
            # We wrap it in a WAV header for easier playback
            with wave.open(temp_filename, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(24000)
                wf.writeframes(audio_data)
            
            self.logger.info(f"Audio pre-generated and saved to: {temp_filename}")
            return temp_filename
        except Exception as e:
            self.logger.error(f"Error during generate_audio: {e}")
            return None

    async def play_audio(self, filename: str, sink: str = None):
        try:
            # Use ffplay to play the wav file
            play_command = ["ffplay", "-nodisp", "-autoexit", filename]
            env = os.environ.copy()
            if sink:
                env["PULSE_SINK"] = sink

            self._current_playback_process = await asyncio.create_subprocess_exec(
                *play_command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            await self._current_playback_process.wait()
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

class RFSTTS(Node):
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
        
        self.client = GeminiTTS(self.get_logger())
        self.load_config()
        self._get_initial_sink_volumes()
        self.loop.create_task(self._playback_worker())

    def _get_initial_sink_volumes(self):
        available_sinks = self._get_available_sinks()
        for sink in available_sinks:
            volume = self._get_raw_sink_volume(sink)
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

    def load_config(self):
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

            available_sinks = self._get_available_sinks()
            self.hdmi_sink = self._find_hdmi_sink(available_sinks) or self.hdmi_sink
            
            if self.chat_mode != 1:
                all_missing = True
                for sink in self.role_map.values():
                    if sink in available_sinks:
                        all_missing = False
                if all_missing and self.role_map:
                    self.use_hdmi_fallback = True

        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    def _find_hdmi_sink(self, available_sinks: list[str]) -> Optional[str]:
        for sink in available_sinks:
            if "hdmi" in sink.lower():
                return sink
        return None

    async def _playback_worker(self):
        while rclpy.ok():
            try:
                role, text, voice_id, is_leader_response, delay, gen_task = await self.playback_queue.get()
                self.current_speaker_role = role

                if delay > 0:
                    await asyncio.sleep(delay)

                if is_leader_response:
                    self._restore_volume()
                
                sink = self.hdmi_sink if (self.chat_mode == 1 or self.use_hdmi_fallback) else self.role_map.get(role)

                if sink is None:
                    # Cancel the ongoing generation task to save API usage
                    if not gen_task.done():
                        gen_task.cancel()
                    self.playback_queue.task_done()
                    continue

                # Wait with a timeout to prevent worker lockup
                audio_file = None
                try:
                    audio_file = await asyncio.wait_for(asyncio.shield(gen_task), timeout=65.0)
                except asyncio.TimeoutError:
                    self.get_logger().error(f"Playback worker: synthesis TIMEOUT for {role}. Skipping audio.")
                except Exception as e:
                    self.get_logger().error(f"Playback worker: generation task failed for {role}: {e}")

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

    def _restore_volume(self):
        volumes_to_restore = self.muted_sinks_original_volumes or self.initial_sink_volumes
        for sink, original_volume in volumes_to_restore.items():
            self._set_sink_volume(sink, original_volume)
        self.muted_sinks_original_volumes.clear()

    def speak_text_callback(self, request, response):
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

            # Start audio generation IMMEDIATELY in the background (Parallel)
            self.get_logger().info(f"Starting background audio synthesis for {role}...")
            gen_task = self.loop.create_task(self.client.generate_audio(text_to_speak, voice_id))
            
            # Put the metadata and the future-task into the playback queue
            self.loop.call_soon_threadsafe(
                self.playback_queue.put_nowait, 
                (role, text_to_speak, voice_id, is_leader_response, request.delay, gen_task)
            )
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in speak_text_callback: {e}")
            response.success = False
        return response

    def interrupt_tts_callback(self, msg: String):
        if msg.data == "stop_all":
            if not self.muted_sinks_original_volumes:
                sinks_to_mute = set([self.hdmi_sink]) if self.chat_mode == 1 else set(self.role_map.values())
                for sink in sinks_to_mute:
                    try:
                        v = self._get_raw_sink_volume(sink)
                        if v is not None: self.muted_sinks_original_volumes[sink] = v
                        self._set_sink_volume(sink, 0)
                    except Exception: pass
            
            while not self.playback_queue.empty():
                try: 
                    item = self.playback_queue.get_nowait()
                    if len(item) > 5 and hasattr(item[5], 'cancel'):
                        item[5].cancel()
                    self.playback_queue.task_done()
                except asyncio.QueueEmpty: break
        elif msg.data == "resume_all":
            self._restore_volume()
        else:
            if self._current_playback_task and not self._current_playback_task.done():
                self._current_playback_task.cancel()
            while not self.playback_queue.empty():
                try: 
                    item = self.playback_queue.get_nowait()
                    if len(item) > 5 and hasattr(item[5], 'cancel'):
                        item[5].cancel()
                    self.playback_queue.task_done()
                except asyncio.QueueEmpty: break
    def destroy_node(self):
        """Explicitly cleanup audio processes on node shutdown."""
        self.get_logger().info("Shutting down TTS node and cleaning up audio processes...")
        if hasattr(self, 'client'):
            self.client.stop()
        super().destroy_node()

    def _get_available_sinks(self) -> list[str]:
        try:
            result = subprocess.run(["pactl", "list", "sinks", "short"], capture_output=True, text=True, check=True, env={"LANG": "C"})
            return [line.split('\t')[1] for line in result.stdout.splitlines() if len(line.split('\t')) > 1]
        except Exception: return []

    def _set_sink_volume(self, sink: str, volume: int):
        try: subprocess.run(["pactl", "set-sink-volume", sink, str(volume)], check=True, env={"LANG": "C"})
        except Exception: pass

    def _get_raw_sink_volume(self, sink: str) -> Optional[int]:
        try:
            result = subprocess.run(["pactl", "list", "sinks"], check=True, capture_output=True, text=True, env={"LANG": "C"})
            m = re.search(rf"Name: {re.escape(sink)}[\s\S]*?Volume:.*?(front-left|mono): (\d+) /", result.stdout)
            return int(m.group(2)) if m else None
        except Exception: return None

    def _restore_volumes_on_exit(self):
        for sink, v in self.muted_sinks_original_volumes.items():
            self._set_sink_volume(sink, v)

def main():
    rclpy.init()
    loop = asyncio.get_event_loop()
    node = RFSTTS(loop)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=lambda: loop.run_forever(), daemon=True)
    t.start()
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node._restore_volumes_on_exit()
        loop.call_soon_threadsafe(loop.stop)
        t.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
