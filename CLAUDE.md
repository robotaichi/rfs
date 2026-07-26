# CLAUDE.md — rfs Repository Refactoring Guide

This file records the guidelines that Claude Code (and any future agent/developer
touching this repository) must follow when preparing **rfs (Robotic Family Simulation)**
for public release on GitHub.

## 0. About this project

`rfs` is a ROS 2 (jazzy) workspace that combines several pseudo-family robots (toio),
voice interaction (STT/TTS), and LLM (Gemini API) based dialogue generation and
psychological evaluation (FACES IV) into a family-therapy simulation.
`ros2 launch rfs_bringup rfs_all.launch.py` (a.k.a. "launch all") starts every node at once.

Package layout:
- `rfs_bringup` — launch files used to start the system
- `rfs_family` — family member conversation state machine, dialogue generation, self-evaluation, reference document delivery
- `rfs_therapist` — combines FACES IV scores, calculates the next target using gradient descent, and draws the plot
- `rfs_stt` — voice activity detection (VAD) on microphone input and speech recognition using the Gemini API
- `rfs_tts` — speech synthesis using the Gemini API, playback/volume control through PulseAudio/PipeWire
- `rfs_toio` / `rfs_toio.toio_speaker_match` — toio cube scanning/connection/movement control, automatic speaker pairing
- `rfs_viewer` — real-time display of the evaluation plot using Tkinter
- `rfs_evaluator_app` — Flask validation web app for researchers (no ROS2 dependency)
- `rfs_interfaces` — custom service definitions (`TTSService`)
- `rfs_config` / `rfs_database` — configuration files and runtime data

## 1. Top priority: "Don't break what already works"

The **single most important rule** for any cleanup work in this repository is:
**never change the behavior that `launch all` currently produces.**

- **Never** change topic names, service names, message types, QoS settings, parameter
  names, or timing constants (sleep durations, timeouts, retry counts, thresholds, etc.),
  file paths, or environment variable names.
- **Never** change the execution order of logic, the order in which locks/threads are
  acquired, or the conditions that trigger a callback.
- Do not change `entry_points` (`console_scripts`) in `setup.py`. Keep each node's
  `main()` in its original file under its original function name. New modules being
  split out should be added as extra files inside the same package directory and
  `import`ed from the original file.
- Do not make a change you cannot verify or are not confident about. **When in doubt,
  don't split the code — just clean up the comments.**
- After every change, check the syntax with `python3 -m py_compile <file>`. Where
  possible, also check that nothing is broken, using `ros2 pkg list` or by
  import-testing the node module directly.

## 2. Refactoring policy: separate the "ROS2 communication layer" from the "logic layer"

The goal is a structure a human can **follow just by reading it**. Each node file is
organized into two kinds of code:

### 2.1 Communication layer (the `Node` class, stays in the original file)
A class inheriting from `rclpy.node.Node`. Its only responsibilities are:
- Creating publishers/subscribers/services/timers in `__init__` (mark this section
  clearly with a comment such as "ROS2 communication setup")
- Callback methods: check/read the incoming message → hand it off to the logic layer → publish the result
- The `main()` entry point

### 2.2 Logic layer (a new file in the same package, no rclpy dependency)
Plain Python classes/functions that do not depend on ROS2. Examples:
- Numeric computation (gradient descent, percentile conversion, etc.)
- File I/O (loading config, appending to the history file, CSV logging)
- External API calls (the Gemini REST API)
- OS-level device control (PulseAudio/PipeWire volume control, BLE scanning, audio device selection)
- Plot generation with matplotlib

The goal is that this logic layer can be unit-tested and reused without ROS2 at all.

### 2.3 What is intentionally left unsplit
Callbacks such as `message_callback` / `publish_pending_scenario` /
`tts_finished_callback` / `move_finished_callback` in `rfs_family_member.py`, where
**"receive message → decide state → publish" is one state change with all its
parts tied closely together**, are **not** forced into a "move to a logic class →
Node receives the result and publishes it" shape.
Reason: this kind of code depends on the exact timing of locks, timers, and making
sure only one thing happens at a time. Splitting it makes it hard to keep the call
order the same, which risks breaking the real behavior of `launch all`. In these
cases the code stays inside the original method, with English comments marking
each step to improve readability instead.

## 3. Comment policy

- Give every class and method a short English docstring (1–3 lines, describing its role concisely).
- Add inline comments only where the reasoning (WHY) is non-obvious. Do not explain
  WHAT the code does if that's already clear from reading it.
- Keep comments in simple, plain English.

## 4. Examples of the split per node (reference)

| Package | What was moved into the logic layer |
|---|---|
| `rfs_therapist` | FACES plot generation (matplotlib), coordinate conversion, FACES table loading, CSV logging → `faces_plotter.py` |
| `rfs_therapist/rfs_evaluator` | Percentile conversion and score calculation → `faces_scoring.py` |
| `rfs_therapist/rfs_optimizer` | Target calculation using gradient descent → `gradient_optimizer.py` |
| `rfs_stt` | VAD recording and Gemini speech recognition (`GeminiLiveRecorder`), microphone device selection → `stt_backend.py` |
| `rfs_tts` | Gemini speech synthesis/playback (`GeminiTTS`), PulseAudio/PipeWire volume control → `tts_backend.py` |
| `rfs_toio` | BLE scanning and toio control utilities → `toio_backend.py` |
| `rfs_toio/toio_speaker_match` | Speaker detection, system TTS (`SystemTTS`), automatic pairing logic → `speaker_matching_backend.py` |
| `rfs_family/rfs_family_member` | Voice assignment, history file I/O, role-name normalization → `family_member_support.py` (state-transition callbacks stay in place) |
| `rfs_evaluator_app` | FACES item definitions and parsing functions → `faces_data.py` |

`rfs_viewer` already separates `PlotViewerGUI` (GUI/display logic) from `RFSViewer`
(ROS2 communication) — this is the ideal shape for this repository.

## 5. Verification steps

After changing a package, run the following before moving on:

```bash
python3 -m py_compile <every file you changed>
```

Where possible, also check that the target node imports cleanly in a ROS2
environment (`/opt/ros/jazzy`, with `rclpy` importable).
