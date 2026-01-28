import os
import json
import random
import shlex
import openai
import sys
import atexit
import datetime
import re
from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
import subprocess
from ament_index_python.packages import get_package_share_directory

# Config paths
HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")

try:
    SHARE_DIR = get_package_share_directory('rfs_config')
    SAVE_DIR = os.path.join(SHARE_DIR, 'config')
except Exception:
    SAVE_DIR = os.path.join(os.path.dirname(DB_DIR), "rfs_config/config")

os.makedirs(DB_DIR, exist_ok=True)

CONFIG_FILE = os.path.join(SAVE_DIR, "config.json")
HISTORY_FILE = os.path.join(DB_DIR, "conversation_history.txt")
TRAJECTORY_FILE = os.path.join(DB_DIR, "evaluation_trajectory.json")

def determine_leader_with_llm(roles: list, theme: str) -> str:
    print("[rfs_launch] Determining leader...")
    if not os.environ.get("OPENAI_API_KEY"):
        return random.choice(roles) if roles else ""
    client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    try:
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": f"Identify the best role to start a conversation about '{theme}' from {roles}."},
                {"role": "user", "content": "Who should start?"}
            ]
        )
        leader = response.choices[0].message.content.strip().lower()
        if leader in [r.lower() for r in roles]: return leader
    except: pass
    return random.choice(roles) if roles else ""

def launch_nodes(context, *args, **kwargs):
    config = kwargs.get('config', {})
    initial_role = kwargs.get('initial_role')
    roles = config.get('family_config', [])
    
    actions = []
    
    # Grid Settings
    GRID_W = 500  # pixels (narrower to avoid overlap)
    GRID_H = 500  # pixels
    TERM_GEOM = "58x18" # character dimensions (shrunk from 75x18)

    # Family Member Nodes (Top Row: Father, Mother, Daughter)
    for i, role in enumerate(roles):
        x_pos = i * GRID_W
        y_pos = 0
        geometry = f"{TERM_GEOM}+{x_pos}+{y_pos}"
        
        inner_cmd = f"ros2 run rfs_family rfs_family_member --role {role}"
        if role == initial_role:
            inner_cmd += " --initiate"
        
        cmd = ['gnome-terminal', '--geometry', geometry, '--', 'bash', '-c', f"source /home/ubuntu/rfs/install/setup.bash; {inner_cmd}; exec bash"]
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))

    # Therapist (Bottom Left: Column 0, Row 1)
    therapist_geometry = f"{TERM_GEOM}+0+{GRID_H}"
    therapist_cmd = ['gnome-terminal', '--geometry', therapist_geometry, '--', 'bash', '-c', 
                     "source /home/ubuntu/rfs/install/setup.bash; ros2 run rfs_therapist rfs_therapist; exec bash"]
    actions.append(ExecuteProcess(cmd=therapist_cmd, output='screen'))

    # STT (Bottom Middle: Column 1, Row 1)
    stt_geometry = f"{TERM_GEOM}+{GRID_W}+{GRID_H}"
    stt_cmd = ['gnome-terminal', '--geometry', stt_geometry, '--', 'bash', '-c', 
               "source /home/ubuntu/rfs/install/setup.bash; ros2 run rfs_stt rfs_stt; exec bash"]
    actions.append(ExecuteProcess(cmd=stt_cmd, output='screen'))

    # Plot Viewer (Bottom Right: Column 2, Row 1 - GUI Only)
    viewer_geometry = f"500x500+{2*GRID_W}+{GRID_H}"
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_viewer', 'rfs_viewer', '--geometry', viewer_geometry], output='log'))

    # Infrastructure (Background)
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_tts', 'rfs_tts'], output='log'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_toio', 'rfs_toio'], output='log'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_evaluation', 'rfs_evaluation'], output='log'))

    return actions

def generate_launch_description():
    with open(CONFIG_FILE, 'r') as f:
        config = json.load(f)
    
    roles = config.get('family_config', [])
    theme = config.get('theme', '')
    
    # --- Pre-launch Cleanup ---
    print("[rfs_launch] Cleaning up previous RFS processes and audio tasks...")
    # Kill any existing ffplay or spd-say processes that might be orphaned
    # Use specific patterns to avoid killing the current launch process
    subprocess.run(["pkill", "-f", "ffplay"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "spd-say"], stderr=subprocess.DEVNULL)
    # Kill RFS nodes specifically by name, avoiding "rfs_bringup" and "rfs_all.launch.py"
    rfs_nodes = ["rfs_family_member", "rfs_therapist", "rfs_stt", "rfs_tts", "rfs_toio", "rfs_evaluation", "rfs_viewer"]
    for node in rfs_nodes:
        subprocess.run(["pkill", "-f", node], stderr=subprocess.DEVNULL)
    # --------------------------

    if os.path.exists(HISTORY_FILE): os.remove(HISTORY_FILE)
    
    # Reset trajectory with S0
    initial_coords = config.get("initial_coords", {"x": 8.0, "y": 8.0})
    initial_trajectory = [{"step": "S0", "target_x": initial_coords.get("x", 8.0), "target_y": initial_coords.get("y", 8.0)}]
    with open(TRAJECTORY_FILE, 'w') as f:
        json.dump(initial_trajectory, f)

    initial_role = determine_leader_with_llm(roles, theme)
    
    return LaunchDescription([
        OpaqueFunction(function=launch_nodes, kwargs={'config': config, 'initial_role': initial_role})
    ])
