import os
import json
import random
import shlex
import openai
import sys
import re
import shutil
import datetime
import time
import atexit
from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
import subprocess
from ament_index_python.packages import get_package_share_directory

# Config paths
HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")

def archive_session_files(context_label="Shutdown"):
    """Stand-alone archival function to preserve session data."""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    archive_dir = os.path.join(DB_DIR, "archive", timestamp)
    
    session_files = {
        "conversation": os.path.join(DB_DIR, "conversation_history.txt"),
        "plot": os.path.join(DB_DIR, "evaluation_plot.png"),
        "trajectory": os.path.join(DB_DIR, "evaluation_trajectory.json"),
        "plot_bg": os.path.join(DB_DIR, "evaluation_plot_bg.png"),
        "history_csv": os.path.join(DB_DIR, "evaluation_history.csv"),
        "debug_log": os.path.join(DB_DIR, "last_evaluation_debug.log")
    }
    
    # Check if ANY of the core files exist
    existing = {name: f for name, f in session_files.items() if os.path.exists(f)}
    
    if existing:
        try:
            os.makedirs(archive_dir, exist_ok=True)
            print(f"\n[rfs_launch][{context_label}] Archiving session to {archive_dir}...")
            
            count = 0
            for name, f_path in existing.items():
                try:
                    dest = os.path.join(archive_dir, os.path.basename(f_path))
                    # Copy then delete for safety
                    shutil.copy2(f_path, dest)
                    if os.path.exists(dest):
                        os.remove(f_path)
                        count += 1
                        print(f"[rfs_launch][{context_label}] Successfully archived {name}")
                except Exception as e:
                    print(f"[rfs_launch][{context_label}] Failed to archive {name}: {e}")
            
            print(f"[rfs_launch][{context_label}] Total {count} files archived.\n")
        except Exception as e:
            print(f"[rfs_launch][{context_label}] Critical error during archival: {e}")
    else:
        if context_label == "Shutdown":
             print(f"\n[rfs_launch][{context_label}] No session data found to archive.\n")

# Register shutdown archival
atexit.register(archive_session_files, "Shutdown")

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

def _build_terminal_cmd(terminal_mode, geometry, inner_cmd):
    """Build a terminal command based on the configured terminal mode."""
    if terminal_mode == "xterm":
        return ['xterm', '-geometry', geometry, '-fa', 'Monospace', '-fs', '10',
                '-hold', '-e', f"bash -c 'source /home/ubuntu/rfs/install/setup.bash; {inner_cmd}'"]
    else:
        # Default: gnome-terminal
        return ['gnome-terminal', '--geometry', geometry, '--', 'bash', '-c',
                f"source /home/ubuntu/rfs/install/setup.bash; {inner_cmd}; exec bash"]

def launch_nodes(context, *args, **kwargs):
    config = kwargs.get('config', {})
    initial_role = kwargs.get('initial_role')
    roles = config.get('family_config', [])
    terminal_mode = config.get('terminal_mode', 'gnome-terminal')
    
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
        
        cmd = _build_terminal_cmd(terminal_mode, geometry, inner_cmd)
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))

    # Therapist (Bottom Left: Column 0, Row 1)
    therapist_geometry = f"{TERM_GEOM}+0+{GRID_H}"
    therapist_cmd = _build_terminal_cmd(terminal_mode, therapist_geometry,
                                        "ros2 run rfs_therapist rfs_therapist")
    actions.append(ExecuteProcess(cmd=therapist_cmd, output='screen'))

    # STT (Bottom Middle: Column 1, Row 1)
    stt_geometry = f"{TERM_GEOM}+{GRID_W}+{GRID_H}"
    stt_cmd = _build_terminal_cmd(terminal_mode, stt_geometry,
                                  "ros2 run rfs_stt rfs_stt")
    actions.append(ExecuteProcess(cmd=stt_cmd, output='screen'))

    # Plot Viewer (Bottom Right: Column 2, Row 1 - GUI Only)
    viewer_geometry = f"500x500+{2*GRID_W}+{GRID_H}"
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_viewer', 'rfs_viewer', '--geometry', viewer_geometry], output='log'))

    # Infrastructure (Background)
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_tts', 'rfs_tts'], output='screen'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_toio', 'rfs_toio'], output='screen'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_therapist', 'rfs_evaluator'], output='screen'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_therapist', 'rfs_optimizer'], output='screen'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_family', 'rfs_generator'], output='screen'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_family', 'rfs_member_evaluator'], output='screen'))
    actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rfs_family', 'rfs_document_processor'], output='screen'))

    return actions

def generate_launch_description():
    # --- Startup Cleanup ---
    archive_session_files("Startup")
    
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
    rfs_nodes = ["rfs_family_member", "rfs_generator", "rfs_member_evaluator", "rfs_document_processor", "rfs_therapist", "rfs_evaluator", "rfs_optimizer", "rfs_stt", "rfs_tts", "rfs_toio", "rfs_viewer"]
    for node in rfs_nodes:
        subprocess.run(["pkill", "-f", node], stderr=subprocess.DEVNULL)
    # --------------------------

    initial_role = determine_leader_with_llm(roles, theme)
    
    return LaunchDescription([
        OpaqueFunction(function=launch_nodes, kwargs={'config': config, 'initial_role': initial_role})
    ])
