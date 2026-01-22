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
graph TD
    %% Define Nodes
    Ther["🟡 rfs_therapist (Orchestrator)"]
    Fam["🟢 rfs_family (Agents)"]
    User(("👤 User (Therapist)"))
    STT["🔵 rfs_stt (Input)"]
    TTS["🔵 rfs_tts (Output)"]
    Eval["🟣 rfs_evaluation (Analysis)"]
    Plot["🟣 rfs_viewer (Visuals)"]
    Toio["🔹 rfs_toio (Physical)"]

    %% Flow with Numbered Steps
    Ther -- "1. Start Turn" --> Fam
    Fam -- "2. TTS Request" --> TTS
    TTS -- "3. Audio Output" --> User
    User -- "4. Voice Intervention" --> STT
    STT -- "5. Transcription" --> Fam
    Fam -- "6. Turn Relay" --> Fam
    Fam -- "7. Trigger Eval" --> Ther
    Ther -- "8. Request Eval" --> Fam
    Fam -- "9. Score Data" --> Ther
    Ther -- "10. Aggregate" --> Eval
    Eval -- "11. Results" --> Ther
    Ther -- "12. Plot Update" --> Plot
    Ther -- "13. Move Logic" --> Toio

    %% Node Styling (Light background, bold border, black text)
    style Ther fill:#FFF9C4,stroke:#FBC02D,stroke-width:2px,color:#000
    style Fam fill:#C8E6C9,stroke:#388E3C,stroke-width:2px,color:#000
    style User fill:#F5F5F5,stroke:#9E9E9E,stroke-width:2px,color:#000
    style STT fill:#B3E5FC,stroke:#0288D1,stroke-width:2px,color:#000
    style TTS fill:#B3E5FC,stroke:#0288D1,stroke-width:2px,color:#000
    style Eval fill:#F3E5F5,stroke:#7B1FA2,stroke-width:2px,color:#000
    style Plot fill:#F3E5F5,stroke:#7B1FA2,stroke-width:2px,color:#000
    style Toio fill:#E0F7FA,stroke:#0097A7,stroke-width:2px,color:#000

    %% Link Styling (Colors matching the source nodes)
    linkStyle 0 stroke:#FBC02D,stroke-width:2px
    linkStyle 1 stroke:#388E3C,stroke-width:2px
    linkStyle 2 stroke:#01579B,stroke-width:2px
    linkStyle 3 stroke:#616161,stroke-width:2px
    linkStyle 4 stroke:#01579B,stroke-width:2px
    linkStyle 5 stroke:#388E3C,stroke-width:2px
    linkStyle 6 stroke:#388E3C,stroke-width:2px
    linkStyle 7 stroke:#FBC02D,stroke-width:2px
    linkStyle 8 stroke:#388E3C,stroke-width:2px
    linkStyle 9 stroke:#FBC02D,stroke-width:2px
    linkStyle 10 stroke:#7B1FA2,stroke-width:2px
    linkStyle 11 stroke:#FBC02D,stroke-width:2px
    linkStyle 12 stroke:#FBC02D,stroke-width:2px
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
