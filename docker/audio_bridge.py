#!/usr/bin/env python3
"""
RFS Audio Bridge — WebSocket server that streams audio between
the container's PulseAudio and the user's browser.

Audio Output (TTS → Browser):
  parec (PulseAudio monitor) → WebSocket → Web Audio API → speakers

Audio Input (Browser → STT):
  getUserMedia → WebSocket → pacat → PulseAudio virtual source → sounddevice
"""

import asyncio
import http.server
import json
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path

try:
    import websockets
    from websockets.asyncio.server import serve
except ImportError:
    print("[audio_bridge] ERROR: 'websockets' package not found. Install with: pip install websockets")
    sys.exit(1)

# ── Configuration ─────────────────────────────────────────────────────────────
WS_PORT = 6082
HTTP_PORT = 6083
SAMPLE_RATE = 24000
CHANNELS = 1
FRAME_SIZE = 4800  # 200ms of 24kHz mono s16le = 4800 samples * 2 bytes

HTML_FILE = Path(__file__).parent / "audio_client.html"

# ── Globals ───────────────────────────────────────────────────────────────────
output_clients: set = set()       # Clients receiving TTS audio
pacat_process: subprocess.Popen | None = None


# ── PulseAudio Setup ──────────────────────────────────────────────────────────
def setup_pulseaudio_virtual_devices():
    """Create a virtual source (mic) so browser audio routes to sounddevice."""
    cmds = [
        # Virtual source for mic input from browser
        ["pactl", "load-module", "module-pipe-source",
         "source_name=browser_mic",
         f"rate={SAMPLE_RATE}", f"channels={CHANNELS}", "format=s16le",
         "file=/tmp/browser_mic_pipe",
         "source_properties=device.description=BrowserMicrophone"],
        # Set browser_mic as default source so sounddevice picks it up
        ["pactl", "set-default-source", "browser_mic"],
    ]

    # Create the named pipe first
    pipe_path = "/tmp/browser_mic_pipe"
    if not os.path.exists(pipe_path):
        os.mkfifo(pipe_path)

    for cmd in cmds:
        try:
            subprocess.run(cmd, capture_output=True, timeout=5)
        except Exception as e:
            print(f"[audio_bridge] Warning: {' '.join(cmd)} failed: {e}")

    print("[audio_bridge] PulseAudio virtual devices configured.")


# ── Audio Output: PulseAudio Monitor → WebSocket ─────────────────────────────
async def audio_output_loop():
    """Capture PulseAudio output via parec and broadcast to WebSocket clients."""
    global output_clients
    while True:
        if len(output_clients) == 0:
            await asyncio.sleep(0.2)
            continue

        proc = None
        try:
            proc = await asyncio.create_subprocess_exec(
                "parec",
                "--format=s16le",
                f"--rate={SAMPLE_RATE}",
                f"--channels={CHANNELS}",
                "--device=@DEFAULT_SINK@.monitor",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )

            while True:
                data = await proc.stdout.read(FRAME_SIZE * 2)  # 2 bytes per sample
                if not data:
                    break
                if len(output_clients) == 0:
                    break
                # Send binary PCM data to all connected clients
                dead = []
                for ws in list(output_clients):
                    try:
                        await ws.send(data)
                    except Exception:
                        dead.append(ws)
                for ws in dead:
                    output_clients.discard(ws)

        except Exception as e:
            print(f"[audio_bridge] parec error: {e}")
            await asyncio.sleep(1)
        finally:
            if proc and proc.returncode is None:
                proc.kill()


# ── Audio Input: WebSocket → PulseAudio Pipe Source ───────────────────────────
mic_pipe_fd = None

def open_mic_pipe():
    """Open the mic pipe for writing (non-blocking at open, blocking writes)."""
    global mic_pipe_fd
    if mic_pipe_fd is None:
        try:
            mic_pipe_fd = os.open("/tmp/browser_mic_pipe", os.O_WRONLY | os.O_NONBLOCK)
        except OSError:
            mic_pipe_fd = None
    return mic_pipe_fd


# ── WebSocket Handler ─────────────────────────────────────────────────────────
async def handle_client(websocket):
    """Handle a WebSocket connection for bidirectional audio."""
    client_addr = websocket.remote_address
    print(f"[audio_bridge] Client connected: {client_addr}")
    output_clients.add(websocket)

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                # Mic data from browser → write to PulseAudio pipe source
                fd = open_mic_pipe()
                if fd is not None:
                    try:
                        os.write(fd, message)
                    except OSError:
                        pass
            elif isinstance(message, str):
                # Control messages (JSON)
                try:
                    msg = json.loads(message)
                    if msg.get("type") == "ping":
                        await websocket.send(json.dumps({"type": "pong"}))
                    elif msg.get("type") == "config":
                        await websocket.send(json.dumps({
                            "type": "config",
                            "sampleRate": SAMPLE_RATE,
                            "channels": CHANNELS,
                        }))
                except json.JSONDecodeError:
                    pass

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        output_clients.discard(websocket)
        print(f"[audio_bridge] Client disconnected: {client_addr}")


# ── HTTP Server for audio_client.html ─────────────────────────────────────────
def start_http_server():
    """Serve audio_client.html on HTTP_PORT."""
    class Handler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html", "/audio.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(HTML_FILE.read_bytes())
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, format, *args):
            pass  # Suppress HTTP logs

    server = http.server.HTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[audio_bridge] Audio control page: http://localhost:{HTTP_PORT}")
    server.serve_forever()


# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    setup_pulseaudio_virtual_devices()

    # Start HTTP server in background thread
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    # Start WebSocket server
    print(f"[audio_bridge] WebSocket server starting on port {WS_PORT}...")
    async with serve(handle_client, "0.0.0.0", WS_PORT):
        # Start audio output capture loop
        await audio_output_loop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[audio_bridge] Shutting down.")
