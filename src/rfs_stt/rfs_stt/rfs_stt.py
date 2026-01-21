#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import time
import sounddevice as sd
import os
import numpy as np
from google import genai
from google.genai import types
from collections import deque
import asyncio
import webrtcvad
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class GeminiLiveRecorder:
    def __init__(
        self,
        model: str = "gemini-2.5-flash-native-audio-preview-12-2025",
        api_key_env: str = "GEMINI_API_KEY",
        vad_aggressiveness: int = 3,
        silence_duration_s: float = 2.0,
        speech_trigger_frames: int = 5,
        on_start: callable = lambda: None,
        on_end: callable = lambda: None,
        on_speech_status_change: callable = lambda x: None,
        logger=None,
    ):
        self.logger = logger
        if not os.environ.get(api_key_env):
            self.logger.error("GEMINI_API_KEY environment variable is not set.")
        self.client = genai.Client(api_key=os.environ.get(api_key_env))
        self.model = model
        self.config = {
            "response_modalities": ["TEXT"],
            "realtime_input_config": {"automatic_activity_detection": {"disabled": True}, "activity_handling": "NO_INTERRUPTION"},
            "input_audio_transcription": {},
        }
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

    async def _record_and_send(self, session):
        speech_started = False
        silence_counter = 0
        speech_frame_counter = 0
        pre_buffer = deque(maxlen=self.speech_trigger_frames + 5)

        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=self.frame_size,
                dtype="int16",
                channels=1,
            ) as mic:
                self.logger.info("RFS STT: Listening...")
                while rclpy.ok():
                    frame, _ = mic.read(self.frame_size)
                    is_speech = self.vad.is_speech(frame, self.sample_rate)

                    if is_speech and not self._is_speech_active:
                        self._is_speech_active = True
                        self.on_speech_status_change(True)
                    elif not is_speech and self._is_speech_active:
                        self._is_speech_active = False
                        self.on_speech_status_change(False)

                    if not speech_started:
                        pre_buffer.append(frame)
                        if is_speech:
                            speech_frame_counter += 1
                            if speech_frame_counter >= self.speech_trigger_frames:
                                self.on_start()
                                speech_started = True
                                await session.send_realtime_input(activity_start=types.ActivityStart())
                                for p_frame in pre_buffer:
                                    await session.send_realtime_input(audio=types.Blob(data=bytes(p_frame), mime_type=f"audio/pcm;rate={self.sample_rate}"))
                                pre_buffer.clear()
                        else:
                            speech_frame_counter = 0
                    else:
                        await session.send_realtime_input(audio=types.Blob(data=bytes(frame), mime_type=f"audio/pcm;rate={self.sample_rate}"))
                        if is_speech: silence_counter = 0
                        else: silence_counter += 1
                        
                        if silence_counter > self.max_silence_frames:
                            self.on_end()
                            await session.send_realtime_input(activity_end=types.ActivityEnd())
                            break
        except Exception as e:
            self.logger.error(f"Error in STT recorder: {e}")

    async def _receive_transcript(self, session) -> str:
        buffer = ""
        try:
            async for msg in session.receive():
                if msg.server_content.input_transcription:
                    buffer += msg.server_content.input_transcription.text
        except Exception: pass
        return "".join(buffer.split())

    async def record_and_transcribe(self) -> str:
        try:
            async with self.client.aio.live.connect(model=self.model, config=self.config) as session:
                await self._record_and_send(session)
                return await self._receive_transcript(session)
        except Exception as e:
            self.logger.error(f"Failed to connect to Gemini Live: {e}")
        return ""

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

        self.recorder = GeminiLiveRecorder(
            on_start=self._on_speech_start,
            on_end=self._on_speech_end,
            on_speech_status_change=self._on_speech_status_change,
            logger=self.get_logger()
        )
        self.recorder_thread = threading.Thread(target=self._recorder_loop, daemon=True)
        self.recorder_thread.start()

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

    def _recorder_loop(self):
        self.ready_event.wait()
        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)
        async def run():
            while rclpy.ok():
                transcript = await self.recorder.record_and_transcribe()
                if transcript.strip():
                    print(f"\n[Recognized] User: {transcript.strip()}\n")
                    self.intervention_pub.publish(String(data=f'user: "{transcript.strip()}"'))
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
