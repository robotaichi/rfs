# RFS: Robot Family System

RFS (Robot Family System) is a ROS2-based research and educational simulation platform for family therapy and family psychology. It leverages Multiple LLM-based agents to simulate complex family dynamics, visualizes psychological states on the FACES IV circumplex model, and uses Gradient Descent to suggest therapeutic interventions.

## 🌟 Key Features

- **Multi-Agent Simulation**: Simulates distinct family member personalities (Father, Mother, Daughter, Son) using advanced LLMs.
- **FACES IV Visualization**: Real-time mapping of family dynamics onto Cohesion and Flexibility axes.
- **Dual Trajectory Tracking**: Visualizes both the "Actual Family State" and the "Therapeutic Target" on the same plot.
- **Predictive Interaction**: Implements "Background Scenario Generation" to pre-generate agent responses, significantly reducing latency.
- **Physical Representation**: Integration with [toio™](https://toio.io/) robots for tangible representation of interpersonal distances.
- **Interactive Audio**: Real-time Speech-to-Text (STT) and Text-to-Speech (TTS) capabilities.

## 🏗 System Architecture & Processing Flow

The system operates in a closed-loop cycle consisting of three main phases: **Initialization**, **Interaction**, and **Evaluation/Steering**.

```mermaid
sequenceDiagram
    autonumber
    
    %% Participant Definitions with Vibrant, High-Contrast Colors
    participant User as User #F5F5F5
    participant STT as rfs_stt #81D4FA
    participant Fam as rfs_family_member (N) #A5D6A7
    participant Ther as rfs_therapist #FFEB3B
    participant Eval as rfs_evaluation #E1BEE7
    participant Plot as rfs_viewer #E1BEE7
    participant Toio as rfs_toio #80DEEA
    participant TTS as rfs_tts #81D4FA

    Note over Ther, Fam: phase 1: Initialization
    Ther->>Fam: [Launch] Initiate first turn
    
    Note over User, TTS: phase 2: Interaction Loop (Turns 1-10)
    Fam->>TTS: Request Speech (Service: TTSService)
    TTS-->>User: Audio Output
    Fam->>Fam: Background Generation (Next Turn)
    Fam->>Fam: Turn Relay (rfs_family_actions)
    User->>STT: Voice Intervention
    STT->>Fam: User Input (rfs_stt_result)
    
    Note over Fam, Ther: phase 3: Evaluation & Steering
    Fam->>Ther: Trigger Evaluation (Turn counter reached)
    Ther->>Fam: Request Member Self-Eval (rfs_request_member_evaluation)
    Fam-->>Ther: Member Results (rfs_member_evaluation_results)
    Ther->>Ther: Aggregate & Calculate FACES IV Score
    Ther->>Ther: Gradient Descent for Therapeutic Target
    Ther->>Plot: Update Visuals (rfs_faces_plot_updated)
    Ther->>Toio: Move Robots (rfs_toio_move_script)
    Ther->>Fam: Resume next Step (Step ID)
```

### Detailed Node Responsibilities

| Node Category | Description | Primary Processing |
| :--- | :--- | :--- |
| **Orchestrator** (`rfs_therapist`) | The system brain. Manages step-level logic and therapeutic interventions. | Aggregate member evaluations, calculate Cohesion/Flexibility percentiles, and perform Gradient Descent toward balanced center (50, 50). |
| **Agents** (`rfs_family`) | Individual nodes for each family role (Father, Mother, etc.). | LLM-based response generation, turn-taking logic, and individual FACES IV self-scoring. |
| **Sensory/Motor** (`rfs_stt`, `rfs_tts`, `rfs_toio`) | The physical/audio interface layers. | GEMINI-based speech recognition, multi-sink synchronized audio output, and Bluetooth BLE control for toio robots. |
| **Visualization** (`rfs_viewer`, `rfs_evaluation`) | Real-time monitoring and mapping. | Tkinter-based GUI for plotting the circumplex model and background processing of psychological metrics. |

## ⚙️ Configuration & Environment

### Environment Variables
The system requires valid API keys for LLM and STT functionalities.

- **`OPENAI_API_KEY`**: Used by `rfs_family` for personality simulation and `rfs_evaluation` for mapping family dynamics.
- **`GEMINI_API_KEY`**: Used by `rfs_stt` for high-performance audio transcription and real-time interaction.

### `config.json` Specification
Located in `src/rfs_config/config/config.json`.

| Parameter | Type | Description |
| :--- | :--- | :--- |
| **`theme`** | String | The scenario or topic of conversation (e.g., "Christmas", "Moving Out"). |
| **`w1`, `w2`, `w3`** | Float | Weights for the FACES IV evaluation model (Cohesion, Flexibility, Communication). |
| **`turns_per_step`** | Integer | Number of conversation turns before an evaluation trigger. |
| **`toio_speaker_match`** | List | Hardware mapping for robots (`toio_id`) and audio outputs (`speaker_id`). |

## 🚀 Getting Started

### Prerequisites
- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS2**: [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- **Hardware**: toio™ Core Cubes (Optional).

### Installation
1. **Clone & Build**:
   ```bash
   git clone https://github.com/robotaichi/rfs.git
   cd rfs
   mkdir -p src/rfs_database
   colcon build
   source install/setup.bash
   ```

2. **Launch**:
   ```bash
   ros2 launch rfs_bringup rfs_all.launch.py
   ```

## 📊 FACES IV Model & Gradient Descent
The system identifies the family's position on the Cohesion/Flexibility spectrum. If the state is "Disengaged" or "Enmeshed", the `rfs_therapist` calculates a vector toward the **Balanced Center** and adjusts the behavioral "Steering Prompts" for individual family members to encourage healthier interaction patterns.

## 📜 License
This project is licensed under the MIT License.
