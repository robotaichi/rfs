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
import json
import os
import subprocess
import sys

try:
    import websockets
    from websockets.asyncio.server import serve
except ImportError:
    print("[audio_bridge] ERROR: 'websockets' package not found. Install with: pip install websockets")
    sys.exit(1)

# ── Configuration ─────────────────────────────────────────────────────────────
WS_PORT = 6082
SAMPLE_RATE = 44100
CHANNELS = 1
FRAME_SIZE = 2205  # 20ms of 48kHz mono s16le = 960 samples * 2 bytes

# ── Globals ───────────────────────────────────────────────────────────────────
output_clients: set = set()       # Clients receiving TTS audio
pacat_process: subprocess.Popen | None = None
stats = {"out_bytes": 0, "in_bytes": 0}

async def stats_loop():
    """Log throughput statistics every 5 seconds."""
    while True:
        await asyncio.sleep(5)
        if stats["out_bytes"] > 0 or stats["in_bytes"] > 0:
            print(f"[audio_bridge] Stats (5s): TTS Server->Browser: {stats['out_bytes']} bytes, Mic Browser->Server: {stats['in_bytes']} bytes")
            stats["out_bytes"] = 0
            stats["in_bytes"] = 0


# ── PulseAudio Setup ──────────────────────────────────────────────────────────
def get_default_sink_monitor():
    """Find the monitor source of the default sink."""
    try:
        # Get default sink name
        res = subprocess.run(["pactl", "get-default-sink"], capture_output=True, text=True)
        if res.returncode != 0:
            print(f"[audio_bridge] Error getting default sink: {res.stderr}")
            return None
        default_sink = res.stdout.strip()
        
        # Get monitor source for this sink
        # Simple heuristic: append .monitor
        # But let's verify if it exists
        candidate = f"{default_sink}.monitor"
        
        # Check if candidate exists in sources
        res2 = subprocess.run(["pactl", "list", "sources", "short"], capture_output=True, text=True)
        if candidate in res2.stdout:
            return candidate
            
        print(f"[audio_bridge] Warning: Monitor source {candidate} not found in source list. Using default sink name hoping for best.")
        return candidate
    except Exception as e:
        print(f"[audio_bridge] Error finding monitor source: {e}")
        return None

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
            device = get_default_sink_monitor() or 'auto_null.monitor'
            print(f"[audio_bridge] Starting parec on device: {device}")
            proc = await asyncio.create_subprocess_exec(
                "parec",
                "--format=s16le",
                f"--rate={SAMPLE_RATE}",
                f"--channels={CHANNELS}",
                f"--device={device}",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

            while True:
                data = await proc.stdout.read(FRAME_SIZE * 2)  # 2 bytes per sample
                if not data:
                    stderr = await proc.stderr.read()
                    print(f"[audio_bridge] parec stopped. Stderr: {stderr.decode().strip()}")
                    break
                if len(output_clients) == 0:
                    break
                # Send binary PCM data to all connected clients
                dead = []
                for ws in list(output_clients):
                    try:
                        await ws.send(data)
                        stats["out_bytes"] += len(data)
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
    print(f"[audio_bridge] Incoming connection attempt from: {client_addr}")
    output_clients.add(websocket)
    print(f"[audio_bridge] Client connected: {client_addr}. Active clients: {len(output_clients)}")

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                # Mic data from browser → write to PulseAudio pipe source
                fd = open_mic_pipe()
                if fd is not None:
                    try:
                        os.write(fd, message)
                        stats["in_bytes"] += len(message)
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


# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    setup_pulseaudio_virtual_devices()

    # Start WebSocket server
    print(f"[audio_bridge] WebSocket server starting on port {WS_PORT} (all interfaces)...")
    async with serve(handle_client, None, WS_PORT):
        # Start audio output capture loop
        asyncio.create_task(stats_loop())
        await audio_output_loop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[audio_bridge] Shutting down.")
