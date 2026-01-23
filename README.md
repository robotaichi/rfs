[**English**](README.md) | [**日本語**](README.ja.md)

# RFS: Robot Family System

RFS (Robot Family System) is a ROS2-based research and educational simulation platform for family therapy and family psychology. It leverages Multiple LLM-based agents to simulate complex family dynamics, visualizes psychological states on the FACES IV circumplex model, and uses Gradient Descent to suggest AI-driven therapeutic interventions.

This project is developed as part of the research at the **Fumihide Tanaka Laboratory** at the University of Tsukuba. Our research focuses on the intersection of human-robot interaction, social psychology, and advanced AI to design systems that enhance human well-being and social harmony.

🔗 **Learn more about our research**: [Fumihide Tanaka Laboratory - Projects](https://www.ftl.iit.tsukuba.ac.jp/projects/)

## 🌟 Key Features

- **Multi-Agent Simulation**: Simulates distinct family member personalities (Father, Mother, Daughter, Son) using advanced LLMs.
- **FACES IV Visualization**: Real-time mapping of family dynamics onto Cohesion and Flexibility axes.
- **Dual Trajectory Tracking**: Visualizes both the "Actual Family State" and the "Therapeutic Target" on the same plot.
- **Predictive Interaction**: Implements "Background Scenario Generation" to pre-generate agent responses, significantly reducing latency.
- **Physical Representation**: Integration with [toio™](https://toio.io/) robots for tangible representation of interpersonal distances.
- **Interactive Audio**: Real-time Speech-to-Text (STT) and Text-to-Speech (TTS) capabilities for optional human intervention.

## 🏗 System Architecture & Processing Flow

The system operates in a closed-loop cycle where the **AI Therapist** (`rfs_therapist`) steers family members toward a healthy, balanced state according to Olson's Circumplex Model. The **Human User** can optionally intervene in the family dialogue to influence the simulation.

![System Architecture](docs/images/architecture.png)

### Detailed Node Responsibilities

| Node | Responsibility | Key Function |
| :--- | :--- | :--- |
| **`rfs_therapist`** | Therapeutic Steering | Promotes transitions toward the Balanced Center using Gradient Descent. |
| **`rfs_family`** | Family Members | Simulates personalities (Father, Mother, etc.) using LLMs. |
| **`rfs_stt`** | Audio Input | Real-time Speech-to-Text for human intervention via Gemini Live. |
| **`rfs_tts`** | Audio Output | Multi-sink synchronized Text-to-Speech for family dialogue. |
| **`rfs_toio`** | Physical Layer | Interpersonal distance representation using [toio™](https://toio.io/) robots. |
| **`rfs_viewer`** | Visualization | Real-time GUI for plotting psychological trajectories. |
| **`rfs_evaluation`** | Map & Assess | Aggregates logs and maps them to the FACES IV circumplex. |

## 🚀 Getting Started

### Prerequisites
- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS2**: [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- **Hardware**: [toio™](https://toio.io/) Core Cubes (Optional).

### Requirements

Before building the project, ensure you have the necessary system and Python libraries installed:

**1. System Dependencies**
```bash
sudo apt update && sudo apt install -y python3-tk libportaudio2
```

**2. Python Libraries**
```bash
pip install openai google-genai numpy sounddevice webrtcvad matplotlib toio-py Pillow
```

### Installation

1. **Clone & Build**:
   ```bash
   git clone https://github.com/robotaichi/rfs.git
   cd rfs
   colcon build
   source install/setup.bash
   ```

### Configuration

The system requires valid API keys. For a persistent setup, add them to your `~/.bashrc`:

```bash
# 1. Open .bashrc
nano ~/.bashrc

# 2. Add these lines at the end of the file
export OPENAI_API_KEY="sk-..."
export GEMINI_API_KEY="AIza..."

# 3. Save and reload
source ~/.bashrc
```

- [**`OPENAI_API_KEY`**](https://platform.openai.com/api-keys): Essential for LLM-based dialogue generation and psychological mapping.
- [**`GEMINI_API_KEY`**](https://aistudio.google.com/app/apikey): Required for Gemini Live-based audio transcription.

2. **Launch**:
   ```bash
   ros2 launch rfs_bringup rfs_all.launch.py
   ```

## ⚙️ Settings

### `config.json` Specification
Located in `src/rfs_config/config/config.json`.

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| **`language`** | String | `"en"` | Interaction language: `"en"` (English) or `"ja"` (Japanese). |
| **`theme`** | String | N/A | High-level topic of family conversation. |
| **`chat_mode`** | Integer | `0` | `0` (Normal), `1` (Terminal mode - no hardware). |
| **`toio_move`** | Integer | `0` | Enable toio movement: `1` (Enabled), `0` (Disabled). |
| **`target_user`** | String | `"User"` | Name of the person interacting with the family. |
| **`family_config`** | List | `["father", "mother", "daughter"]` | Active family roles for the simulation. |
| **`toio_speaker_match`** | List | `[...]` | Hardware toio-to-speaker mapping configuration. |
| **`learning_rate_scaling`** | Float | `0.25` | Base therapeutic steering sensitivity. |
| **`w1`, `w2`, `w3`** | Float | `1.0, 1.0, 0.5` | Weights for Cohesion, Flexibility, and Communication. |
| **`turns_per_step`** | Integer | `10` | Frequency of evaluation triggers (Unit: **Turns**). |
| **`vad_aggressiveness`** | Integer | `3` | VAD sound filtering sensitivity (Range: 0-3). |
| **`silence_duration_s`** | Float | `2.0` | Required silence to end speech (Unit: **Seconds**). |
| **`speech_trigger_frames`** | Integer | `5` | Frames required to trigger recording (30ms/frame). |
| **`vad_debug`** | Boolean | `false` | Enable verbose VAD logging for diagnostics. |
| **`vad_energy_threshold`** | Float | `2000.0` | Minimum RMS energy level for speech detection. |
| **`llm_model`** | String | `"gpt-4o"` | Model for family dialogue generation. |
| **`llm_temperature`** | Float | `1.0` | Creativity factor for dialogue. |
| **`llm_evaluation_model`** | String | `"gpt-4o"` | Model for FACES IV self-assessment. |
| **`llm_evaluation_temperature`** | Float | `0.7` | Stability factor for assessment. |
| **`initial_coords`** | Object | `{"x": 8, "y": 8}` | Starting coordinates (Scale: **0-100 Percentile**). |
| **`experiment`** | String | `""` | Optional tag for trial/experiment labeling. |
| **`terminal_mode`** | String | `"gnome-terminal"` | Terminal used for launching nodes. |
| **`shutdown_timer_minutes`** | Integer | `0` | Auto-shutdown timer. `0` (Disabled - infinite), `1+` (Minutes before shutdown). |

### LLM Selection & Guidance
- **Default Model (`gpt-4o`)**: We use `gpt-4o` as the standard for its exceptional reasoning capabilities and nuanced understanding of human social dynamics. It effectively simulates the complex psychological archetypes required for this research.
- **Temperature Settings**:
  - **Dialogue (`1.0`)**: A higher temperature is used for turn-taking to ensure natural, varied, and creative conversation that reflects the dynamic nature of family interactions.
  - **Evaluation (`0.7`)**: A slightly lower temperature is used for psychological assessment to ensure reliable and consistent scoring while still allowing the LLM to capture the "subjective feel" of the simulated member.

## 📊 FACES IV Model & Gradient Descent

The system treats therapeutic intervention as an optimization problem. If a family state is identified as "Disengaged" or "Enmeshed", the **AI Therapist** calculates the optimal path toward health using **Gradient Descent**.

### Percentile Conversion

Before any mathematical processing or plotting, the system converts the **Raw Scores** obtained from assessments into **Percentile Scores**. This conversion is essential for standardized mapping onto the Circumplex Model and ensures that the Gradient Descent operates on a normalized scale ($0$ to $100$).

The following conversion charts, based on standard FACES IV norms, are used by the system.

#### 1. Balanced & Unbalanced Scales (Cohesion & Flexibility)

| Raw Score | Balanced Percentile (C/F) | Unbalanced Percentile (D/E/R/C) |
| :--- | :--- | :--- |
| **7** | 16 | 10 |
| **8** | 18 | 12 |
| **9** | 20 | 13 |
| **10** | 22 | 14 |
| **11** | 24 | 15 |
| **12** | 25 | 16 |
| **13** | 26 | 18 |
| **14** | 27 | 20 |
| **15** | 28 | 24 |
| **16** | 30 | 26 |
| **17** | 32 | 30 |
| **18** | 35 | 32 |
| **19** | 36 | 34 |
| **20** | 38 | 36 |
| **21** | 40 | 40 |
| **22** | 45 | 45 |
| **23** | 50 | 50 |
| **24** | 55 | 55 |
| **25** | 58 | 60 |
| **26** | 60 | 64 |
| **27** | 62 | 68 |
| **28** | 65 | 70 |
| **29** | 68 | 75 |
| **30** | 70 | 80 |
| **31** | 75 | 85 |
| **32** | 80 | 90 |
| **33** | 82 | 95 |
| **34** | 84 | 98 |
| **35** | 85 | 99 |

#### 2. Family Communication

| Raw Score | Percentile || Raw Score | Percentile |
| :--- | :--- | :--- | :--- | :--- |
| **10-23** | 10 || **37** | 58 |
| **24** | 12 || **38** | 62 |
| **25** | 13 || **39** | 65 |
| **26** | 14 || **40** | 70 |
| **27** | 15 || **41** | 74 |
| **28** | 18 || **42** | 80 |
| **29** | 21 || **43** | 83 |
| **30** | 24 || **44** | 86 |
| **31** | 28 || **45** | 88 |
| **32** | 32 || **46** | 90 |
| **33** | 36 || **47** | 94 |
| **34** | 40 || **48** | 96 |
| **35** | 44 || **49** | 97 |
| **36** | 50 || **50** | 99 |

### Ratio Scores

The system also calculates **Ratio Scores** to assess the overall health of the family system. A Ratio Score greater than 1 typically indicates a healthy, balanced system, while a score less than 1 suggests an unbalanced (unhealthy) system.

These are calculated using the converted **Percentile Scores**:

1. **Cohesion Ratio**
> [!NOTE]
> ```math
> \text{Cohesion Ratio} = \frac{C_{bal}}{(C_{dis} + C_{enm}) / 2}
> ```

2. **Flexibility Ratio**
> [!NOTE]
> ```math
> \text{Flexibility Ratio} = \frac{F_{bal}}{(F_{rig} + F_{cha}) / 2}
> ```

3. **Total Ratio**
> [!NOTE]
> ```math
> \text{Total Ratio} = \frac{\text{Cohesion Ratio} + \text{Flexibility Ratio}}{2}
> ```

### Mathematical Foundation

The AI Therapist calculates the optimal therapeutic path using Gradient Descent on the family state vector.

#### 1. State Vector ($x_t$)
The family state at turn $t$ is represented as a 7-dimensional vector consisting of the converted **Percentile Scores**:

> [!NOTE]
> ```math
> x_t = \begin{bmatrix} C_{bal} \\ C_{dis} \\ C_{enm} \\ F_{bal} \\ F_{rig} \\ F_{cha} \\ Comm \end{bmatrix} = [C_{bal}, \dots, Comm]^T
> ```

#### 2. Objective Function ($J(x_t)$)
The goal is to minimize a cost function that balances the FACES IV Ratio (Health) and Centering (Stability):

> [!NOTE]
> ```math
> J(x_t) = \omega_1 \frac{U}{2B} - \omega_2 Comm_t + \frac{\omega_3}{2} \left[ (x - 50)^2 + (y - 50)^2 \right]
> ```
Where:
- $B = C_{bal} + F_{bal}$ (Balanced Sum)
- $U = C_{dis} + C_{enm} + F_{rig} + F_{cha}$ (Unbalanced Sum)

> [!NOTE]
> ```math
> x = C_{bal} + \frac{C_{enm} - C_{dis}}{2}, \quad y = F_{bal} + \frac{F_{cha} - F_{rig}}{2}
> ```

#### 3. Gradient Calculation ($\nabla J(x_t)$)
The gradient vector $\nabla J(x_t)$ represents the direction of steepest increase for the cost function:

> [!NOTE]
> ```math
> \nabla J(x_t) = \left[ \frac{\partial J}{\partial C_{bal}}, \dots, \frac{\partial J}{\partial Comm} \right]^T
> ```

Individual partial derivatives are calculated as follows (combining the Ratio and Centering terms):

> [!NOTE]
> ```math
> \begin{aligned}
> \frac{\partial J}{\partial C_{bal}} &= - \frac{\omega_1 U}{2B^2} + \omega_3(x - 50) \\
> \frac{\partial J}{\partial F_{bal}} &= - \frac{\omega_1 U}{2B^2} + \omega_3(y - 50) \\
> \frac{\partial J}{\partial C_{enm}} &= \frac{\omega_1}{2B} + \frac{\omega_3}{2}(x - 50) \\
> \frac{\partial J}{\partial C_{dis}} &= \frac{\omega_1}{2B} - \frac{\omega_3}{2}(x - 50) \\
> \frac{\partial J}{\partial F_{cha}} &= \frac{\omega_1}{2B} + \frac{\omega_3}{2}(y - 50) \\
> \frac{\partial J}{\partial F_{rig}} &= \frac{\omega_1}{2B} - \frac{\omega_3}{2}(y - 50) \\
> \frac{\partial J}{\partial Comm} &= - \omega_2
> \end{aligned}
> ```

#### 4. Update Rule & Adaptive Learning Rate
The target state is updated iteratively:

> [!NOTE]
> ```math
> x_{t+1} = x_t - \eta(Comm_t) \cdot \nabla J(x_t)
> ```

Where the adaptive learning rate $\eta (Comm_t)$ represents the **step width**:

> [!NOTE]
> ```math
> \eta(Comm_t) = \frac{Comm_t}{100} \cdot 0.25
> ```

Where **0.25** is the base scaling factor (configurable via `learning_rate_scaling` in `config.json`) that determines the overall sensitivity of the therapeutic steering.

*This step width, along with the Communication dimension, acts as the **driving force** for promoting Cohesion and Flexibility.*

The resulting vector adjusts the **Behavioral Steering Prompts** for individual family members, pulling the system towards the **Balanced Center (50, 50)**.

## 📚 References
- **Olson's Circumplex Model**: [Circumplex Model: An Update (Prepare/Enrich)](https://www.prepare-enrich.com/wp-content/uploads/2022/08/Circumplex-Model-An-Update.pdf)
- **FACES IV Manual**: [FACES IV: Manual de Aplicación de Instrumento](https://www.studocu.com/cl/document/universidad-de-valparaiso/trabajo-social-de-familia/faces-iv-manual-aplicacion-de-instrumento/107365427)

## 📜 License
This project is licensed under the MIT License.
