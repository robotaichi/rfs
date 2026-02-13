#!/bin/bash
###############################################################################
# RFS Docker Entrypoint
# Starts VNC server, noVNC proxy, PulseAudio, and configures RFS for Docker
###############################################################################
set -e

echo "============================================="
echo "  RFS (Robot Family System) Docker Container"
echo "============================================="

# ─── Configuration ────────────────────────────────────────────────────────────
VNC_RESOLUTION="${VNC_RESOLUTION:-1920x1080}"
VNC_DISPLAY=":1"
VNC_PORT=5901
NOVNC_PORT=6080

# ─── Configure RFS for Docker (switch terminal_mode to xterm) ─────────────────
for CONFIG_FILE in \
    /home/ubuntu/rfs/src/rfs_config/config/config.json \
    /home/ubuntu/rfs/install/rfs_config/share/rfs_config/config/config.json; do
    if [ -f "$CONFIG_FILE" ]; then
        python3 -c "
import json
with open('$CONFIG_FILE', 'r') as f:
    config = json.load(f)
config['terminal_mode'] = 'xterm'
with open('$CONFIG_FILE', 'w') as f:
    json.dump(config, f, indent=2, ensure_ascii=False)
print(f'[entrypoint] {\"$CONFIG_FILE\".split(\"/\")[-3]}: terminal_mode set to xterm')
"
    fi
done

# ─── Start PulseAudio ─────────────────────────────────────────────────────────
echo "[entrypoint] Starting PulseAudio..."
pulseaudio --start --exit-idle-time=-1 2>/dev/null || true

# ─── Start VNC server ─────────────────────────────────────────────────────────
echo "[entrypoint] Starting VNC server (${VNC_RESOLUTION})..."
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true

VNC_ARGS="${VNC_DISPLAY} -geometry ${VNC_RESOLUTION} -depth 24 -localhost no -xstartup /home/ubuntu/.vnc/xstartup"
if [ -n "$VNC_PASSWORD" ]; then
    echo "$VNC_PASSWORD" | vncpasswd -f > /home/ubuntu/.vnc/passwd
    chmod 600 /home/ubuntu/.vnc/passwd
    echo "[entrypoint] VNC password set."
else
    echo "[entrypoint] VNC running without password."
    VNC_ARGS="${VNC_ARGS} --SecurityTypes=None --I-KNOW-THIS-IS-INSECURE"
fi

vncserver ${VNC_ARGS} 2>&1 | head -5

export DISPLAY=${VNC_DISPLAY}
sleep 2

# ─── Start noVNC proxy ────────────────────────────────────────────────────────
echo "[entrypoint] Starting noVNC on port ${NOVNC_PORT}..."
websockify --web=/usr/share/novnc ${NOVNC_PORT} localhost:${VNC_PORT} &
NOVNC_PID=$!

sleep 1
echo ""
echo "============================================="
echo "  ✅ RFS is ready!"
echo ""
echo "  🌐 Open in browser: http://localhost:${NOVNC_PORT}/vnc.html"
echo ""
echo "  📋 To launch RFS:"
echo "     Double-click 'RFS Launch' on the desktop"
echo "     or open a terminal and run:"
echo "     ros2 launch rfs_bringup rfs_all.launch.py"
echo ""
if [ -n "$OPENAI_API_KEY" ]; then
    echo "  🔑 OpenAI API Key: SET"
else
    echo "  🔑 OpenAI API Key: NOT SET"
fi
if [ -n "$GEMINI_API_KEY" ]; then
    echo "  🔑 Gemini API Key: SET"
else
    echo "  🔑 Gemini API Key: NOT SET"
fi
echo "============================================="
echo ""

# ─── Keep container running ───────────────────────────────────────────────────
wait $NOVNC_PID
