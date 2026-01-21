# RFS: Robot Family System

RFS (Robot Family System) is a ROS2-based research and educational simulation platform for family therapy and family psychology. It simulates family dynamics using Multiple LLM-based agents and visualizes their psychological states on the FACES IV circumplex model, while steering the family toward healthier communication patterns using Gradient Descent.

## System Architecture

The project consists of several modular ROS2 nodes working in tandem:

- **`rfs_therapist`**: The core orchestrator. It calculates the "Therapeutic Target" using Gradient Descent based on family evaluations and steers the family towards the logical center (50, 50 - Balanced Type).
- **`rfs_family`**: Simulates family members (Father, Mother, Daughter, Son). Each member has a distinct personality and responds to others based on the current family psychological state.
- **`rfs_evaluation`**: Analyzes family communication and maps the family's state to the 6 dimensions of FACES IV (Cohesion, Flexibility, Disengaged, Enmeshed, Rigid, Chaotic).
- **`rfs_viewer`**: A real-time GUI that visualizes the family's trajectory on the FACES IV circumplex plot. It shows actual results vs. therapeutic goals.
- **`rfs_toio`**: Controls [toio™](https://toio.io/) robots to represent family members' psychological "closeness" and "flexibility" in physical space.
- **`rfs_stt` & `rfs_tts`**: Handle audio input (Speech-to-Text) and output (Text-to-Speech) for immersive interaction.
- **`rfs_bringup`**: Contains the central launch file to start the entire system.
- **`rfs_interfaces`**: Defines custom service interfaces (e.g., `TTSService`).
- **`rfs_config`**: Stores static configuration, prompt templates, and personality assets.
- **`rfs_database`**: A persistent storage directory for session histories, evaluation trajectories, and plot images.

## Installation

### Dependencies
- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS2**: [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- **Python**: 3.10+
- **API Key**: An `OPENAI_API_KEY` is required for LLM-based simulation and evaluation.

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/[your-username]/rfs.git
   cd rfs
   ```
2. Install Python dependencies:
   ```bash
   pip install openai matplotlib pillow
   ```
3. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

Start the complete system with a single command:
```bash
ros2 launch rfs_bringup rfs_all.launch.py
```

### Configuration
You can customize the simulation theme and initial family state in `src/rfs_config/config/config.json`.

## Performance Features
- **Sequential Turn Relay**: Minimizes overlap during conversation.
- **Background Scenario Generation**: Pre-generates the next agent's response while the current agent is speaking to reduce latency.
- **Robust Shutdown**: Graceful termination handling for all nodes.

## License
[License Type - e.g., MIT]
