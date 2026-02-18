/**
 * RFS Audio Bridge Overlay — Floating audio widget for noVNC
 *
 * Injects a small floating panel into the VNC page to handle
 * audio streaming between the Docker container and the browser.
 *
 * Audio Output: PulseAudio → parec → WebSocket → Web Audio API → speakers
 * Audio Input: getUserMedia → WebSocket → pacat → PulseAudio virtual source
 */
(function () {
  'use strict';

  // ── Config ─────────────────────────────────────────────────────────────────
  const WS_URL = `ws://${location.hostname}:6082`;
  let SAMPLE_RATE = 48000;
  let CHANNELS = 1;

  // ── State ──────────────────────────────────────────────────────────────────
  let ws = null;
  let audioCtx = null;
  let micStream = null;
  let micProcessor = null;
  let isConnected = false;

  console.log("[RFS Audio] Overlay initialized. Sample Rate:", SAMPLE_RATE);

  // ── Create Overlay UI ──────────────────────────────────────────────────────
  const css = document.createElement('style');
  css.textContent = `
    #rfs-audio-panel {
      position: fixed; bottom: 16px; right: 16px; z-index: 99999;
      font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
      transition: all 0.3s ease;
    }
    #rfs-audio-toggle {
      width: 48px; height: 48px; border-radius: 50%;
      background: #6c63ff; color: white; border: none;
      font-size: 22px; cursor: pointer;
      box-shadow: 0 4px 16px rgba(108,99,255,0.4);
      display: flex; align-items: center; justify-content: center;
      transition: all 0.2s;
    }
    #rfs-audio-toggle:hover {
      transform: scale(1.1);
      box-shadow: 0 6px 24px rgba(108,99,255,0.6);
    }
    #rfs-audio-toggle.connected { background: #34d399; box-shadow: 0 4px 16px rgba(52,211,153,0.4); }
    #rfs-audio-card {
      display: none; position: absolute; bottom: 60px; right: 0;
      background: #1a1d27; border: 1px solid #2a2d3a; border-radius: 12px;
      padding: 16px; width: 280px;
      box-shadow: 0 8px 32px rgba(0,0,0,0.5);
      color: #e1e4ed;
    }
    #rfs-audio-card.open { display: block; }
    #rfs-audio-card h3 { margin: 0 0 12px; font-size: 14px; font-weight: 600; }
    .rfs-status-row {
      display: flex; align-items: center; gap: 8px;
      padding: 6px 10px; background: #0f1117; border-radius: 8px;
      margin-bottom: 6px; font-size: 13px;
    }
    .rfs-dot {
      width: 8px; height: 8px; border-radius: 50%;
      background: #f87171; flex-shrink: 0; transition: all 0.3s;
    }
    .rfs-dot.on { background: #34d399; box-shadow: 0 0 6px #34d399; }
    .rfs-dot.warn { background: #fbbf24; }
    .rfs-label { flex: 1; }
    .rfs-val { font-size: 11px; color: #8b8fa3; }
    #rfs-audio-btn {
      width: 100%; padding: 10px; border: none; border-radius: 8px;
      font-size: 13px; font-weight: 600; cursor: pointer;
      margin-top: 8px; transition: all 0.2s;
      background: #6c63ff; color: white;
    }
    #rfs-audio-btn:hover { background: #5b54e6; }
    #rfs-audio-btn.off { background: #2a2d3a; color: #8b8fa3; }
    #rfs-audio-btn.off:hover { background: #363a4a; }
    .rfs-vis {
      height: 24px; display: flex; align-items: flex-end;
      justify-content: center; gap: 1px; margin: 8px 0 4px;
    }
    .rfs-bar {
      width: 2px; background: #6c63ff; border-radius: 1px;
      transition: height 0.05s; min-height: 1px;
    }
    #raLog { font-size: 10px; color: #f87171; margin-top: 8px; word-break: break-all; }
  `;
  document.head.appendChild(css);

  const panel = document.createElement('div');
  panel.id = 'rfs-audio-panel';
  panel.innerHTML = `
    <div id="rfs-audio-card">
      <h3>🔊 Audio Bridge</h3>
      <div class="rfs-status-row"><div class="rfs-dot" id="raDot1"></div><span class="rfs-label">WebSocket</span><span class="rfs-val" id="raS1">Off</span></div>
      <div class="rfs-status-row"><div class="rfs-dot" id="raDot2"></div><span class="rfs-label">Speaker</span><span class="rfs-val" id="raS2">Off</span></div>
      <div class="rfs-status-row"><div class="rfs-dot" id="raDot3"></div><span class="rfs-label">Mic</span><span class="rfs-val" id="raS3">Off</span></div>
      <div class="rfs-vis" id="raVis"></div>
      <button id="rfs-audio-btn" onclick="window._rfsToggleAudio()">🔊 Connect</button>
      <div id="raLog"></div>
    </div>
    <button id="rfs-audio-toggle" onclick="document.getElementById('rfs-audio-card').classList.toggle('open')" title="Audio Bridge">🔇</button>
  `;
  document.body.appendChild(panel);

  // Refs
  const toggle = document.getElementById('rfs-audio-toggle');
  const dot1 = document.getElementById('raDot1'), s1 = document.getElementById('raS1');
  const dot2 = document.getElementById('raDot2'), s2 = document.getElementById('raS2');
  const dot3 = document.getElementById('raDot3'), s3 = document.getElementById('raS3');
  const btn = document.getElementById('rfs-audio-btn');
  const vis = document.getElementById('raVis');
  const log = document.getElementById('raLog');

  // Visualizer bars
  const NUM_BARS = 24;
  for (let i = 0; i < NUM_BARS; i++) {
    const b = document.createElement('div'); b.className = 'rfs-bar'; b.style.height = '1px';
    vis.appendChild(b);
  }
  const bars = vis.querySelectorAll('.rfs-bar');

  // ── Connection ─────────────────────────────────────────────────────────────
  window._rfsToggleAudio = async function () {
    isConnected ? disconnect() : await connect();
  };

  async function connect() {
    log.textContent = "";
    try {
      console.log("[RFS Audio] Connecting to", WS_URL);
      s1.textContent = 'Connecting...';
      ws = new WebSocket(WS_URL);
      ws.binaryType = 'arraybuffer';

      ws.onopen = () => {
        console.log("[RFS Audio] WebSocket connected");
        dot1.classList.add('on'); s1.textContent = 'Connected';
        ws.send(JSON.stringify({ type: 'config' }));
      };
      ws.onclose = (e) => {
        console.log("[RFS Audio] WebSocket closed", e.code, e.reason);
        dot1.classList.remove('on'); s1.textContent = 'Off';
        if (isConnected) {
          log.textContent = "Connection lost. Retrying...";
          setTimeout(() => { if (isConnected) connect(); }, 2000);
        }
      };
      ws.onerror = (e) => {
        console.error("[RFS Audio] WebSocket error", e);
        s1.textContent = 'Error';
        log.textContent = "WS Error: Could not reach " + WS_URL;
      };
      ws.onmessage = (e) => {
        if (typeof e.data === 'string') {
          const m = JSON.parse(e.data);
          if (m.type === 'config') {
            console.log("[RFS Audio] Server config:", m);
            // We don't change SAMPLE_RATE here as AudioContext is already starting
          }
          return;
        }
        playPCM(e.data);
      };

      console.log("[RFS Audio] Initializing AudioContext at", SAMPLE_RATE);
      audioCtx = new AudioContext({ sampleRate: SAMPLE_RATE });
      dot2.classList.add('on'); s2.textContent = 'Active';

      try {
        micStream = await navigator.mediaDevices.getUserMedia({
          audio: { sampleRate: SAMPLE_RATE, channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true }
        });
        const src = audioCtx.createMediaStreamSource(micStream);
        micProcessor = audioCtx.createScriptProcessor(4096, 1, 1);
        micProcessor.onaudioprocess = (ev) => {
          if (!ws || ws.readyState !== WebSocket.OPEN) return;
          const inp = ev.inputBuffer.getChannelData(0);
          const tLen = Math.round(inp.length * SAMPLE_RATE / audioCtx.sampleRate);
          const pcm = new Int16Array(tLen);

          // Linear interpolation for higher quality resampling if needed
          for (let i = 0; i < tLen; i++) {
            const pos = i * (inp.length / tLen);
            const idx = Math.floor(pos);
            const frac = pos - idx;
            const s1 = inp[idx];
            const s2 = idx + 1 < inp.length ? inp[idx + 1] : s1;
            const s = s1 + (s2 - s1) * frac;
            const clamped = Math.max(-1, Math.min(1, s));
            pcm[i] = clamped < 0 ? clamped * 0x8000 : clamped * 0x7FFF;
          }
          ws.send(pcm.buffer);
        };
        src.connect(micProcessor); micProcessor.connect(audioCtx.destination);
        dot3.classList.add('on'); s3.textContent = 'Active';
      } catch (me) {
        console.error("[RFS Audio] Mic Error:", me);
        dot3.classList.add('warn'); s3.textContent = 'Denied';
        log.textContent = "Mic Error: " + me.message;
      }

      isConnected = true;
      toggle.textContent = '🔊'; toggle.classList.add('connected');
      btn.textContent = '⏹ Disconnect'; btn.className = 'rfs-audio-btn off';
      btn.id = 'rfs-audio-btn';
    } catch (err) {
      console.error("[RFS Audio] General Error:", err);
      log.textContent = "Init Error: " + err.message;
      disconnect();
    }
  }

  function disconnect() {
    isConnected = false;
    if (micProcessor) { micProcessor.disconnect(); micProcessor = null; }
    if (micStream) { micStream.getTracks().forEach(t => t.stop()); micStream = null; }
    if (audioCtx) { audioCtx.close(); audioCtx = null; }
    if (ws) { ws.close(); ws = null; }
    dot1.classList.remove('on'); dot2.classList.remove('on'); dot3.classList.remove('on', 'warn');
    s1.textContent = s2.textContent = s3.textContent = 'Off';
    toggle.textContent = '🔇'; toggle.classList.remove('connected');
    btn.textContent = '🔊 Connect'; btn.className = '';
    btn.id = 'rfs-audio-btn';
    bars.forEach(b => b.style.height = '1px');
  }

  // ── PCM Playback ─────────────────────────────────────────────────────────
  let nextTime = 0;
  const JITTER_BUFFER = 0.1; // 100ms buffer to prevent cracking

  function playPCM(buf) {
    if (!audioCtx) return;

    // Create Float32 buffer
    const i16 = new Int16Array(buf);
    const f32 = new Float32Array(i16.length);
    for (let i = 0; i < i16.length; i++) f32[i] = i16[i] / 32768.0;

    updateVis(f32);

    if (audioCtx.state === 'suspended') audioCtx.resume();

    const buffer = audioCtx.createBuffer(1, f32.length, SAMPLE_RATE);
    buffer.getChannelData(0).set(f32);

    const source = audioCtx.createBufferSource();
    source.buffer = buffer;
    source.connect(audioCtx.destination);

    const now = audioCtx.currentTime;

    // If nextTime is in the past (underrun), reset it to now + jitter buffer
    if (nextTime < now) {
      nextTime = now + JITTER_BUFFER;
    }

    source.start(nextTime);
    nextTime += buffer.duration;
  }

  function updateVis(samples) {
    const step = Math.floor(samples.length / NUM_BARS);
    for (let i = 0; i < NUM_BARS; i++) {
      let sum = 0;
      for (let j = 0; j < step; j++) sum += Math.abs(samples[i * step + j] || 0);
      bars[i].style.height = Math.max(1, Math.min(22, (sum / step) * 300)) + 'px';
    }
    setTimeout(() => bars.forEach(b => { b.style.height = Math.max(1, parseFloat(b.style.height) * 0.7) + 'px'; }), 100);
  }
})();
