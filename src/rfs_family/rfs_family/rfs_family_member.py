#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String, Bool
import sys
import os
import os
import json
import time
import argparse
import random
import csv
import io
import fcntl
import threading
import signal
import shutil
from datetime import datetime
from rfs_interfaces.srv import TTSService
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from ament_index_python.packages import get_package_share_directory
import openai
from openai import OpenAI

# Constants
HOME = os.path.expanduser("~")
DB_DIR = os.path.join(HOME, "rfs/src/rfs_database")

try:
    SHARE_DIR = get_package_share_directory('rfs_config')
    SAVE_DIR = os.path.join(SHARE_DIR, 'config')
except Exception:
    SAVE_DIR = os.path.join(os.path.dirname(DB_DIR), "rfs_config/config")

os.makedirs(DB_DIR, exist_ok=True)

HISTORY_FILE = os.path.join(DB_DIR, "conversation_history.txt")
CONFIG_FILE = os.path.join(SAVE_DIR, "config.json")
VOICE_LIST_FILE = os.path.join(SAVE_DIR, "voice_list.txt")
USER_INTERVENTION_LOCK_FILE = os.path.join(DB_DIR, "user_intervention_lock.txt")
TRAJECTORY_FILE = os.path.join(DB_DIR, "evaluation_trajectory.json")

SINGLE_MEMBER_ROLE = 'androgynous_communication_robot'
GLASS_CASTLE_FILE = os.path.join(DB_DIR, "glass_castle_analysis.md")
SINGLE_MEMBER_VOICE = {
    "id": "23",
    "name": "WhiteCUL",
    "style": "Normal",
    "reason": "To match soft and neutral communication."
}


# OpenAI Client
if not os.environ.get("OPENAI_API_KEY"):
    print("Please set the OPENAI_API_KEY environment variable.")
    sys.exit(1)
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

class TTSClient:
    def __init__(self, node_name):
        self.node = Node(f'rfs_tts_client_{node_name}')
        self.cli = self.node.create_client(TTSService, 'rfs_speak_text')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().info('Waiting for rfs_speak_text service...')
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        t = threading.Thread(target=self.exec.spin, daemon=True)
        t.start()

    def speak(self, text: str, delay: float = 0.0):
        req = TTSService.Request()
        req.text = text
        req.delay = delay
        self.cli.call_async(req)

class RFSFamilyMember(Node):
    def __init__(self, role: str, theme: str, chat_mode: int, target_user: str, move: int, family_config: list, initial_role: str = ""):
        orig_role = role.lower()
        super().__init__('rfs_family_member_' + orig_role)
        
        self.role = orig_role
        self.theme = theme
        self.chat_mode = chat_mode
        self.target_user = target_user
        self.move_enabled = move
        self.family_config = [m.lower() for m in family_config]
        self.initial_role = initial_role.lower()
        self.start_pending = (self.initial_role == self.role)
        
        self.tts_ready = False
        self.toios_ready = False
        self.initialization_done = False
        self.is_scenario_generation_paused = False
        self.expecting_intervention_resolution = False
        self.am_i_intervention_responder = False
        self.robot_positions = {}
        self.pending_move_command = None
        self.pending_scenario_conversation = None
        self.pending_scenario_move = None
        self.is_user_intervention_active_for_publishing_block = False
        self.pending_delay = 1.0
        self.next_generation_queued = False  # One-ahead queue guard: only one background generation at a time
        self.audio_synthesis_requested = False
        self.step_count = 0
        self.waiting_for_evaluation = False
        self.turns_per_step = 10
        self.initial_coords = {"x": 8.0, "y": 8.0}
        self.pending_intervention_text = None
        self.pending_relay_recipient = None
        self.pending_eval_step_id = None # Track if we are the one to trigger evaluation
        self.language = "en"
        self.llm_model = "gpt-5.2-chat-latest"
        self.llm_temperature = 1.0
        self.llm_evaluation_model = "gpt-5.2-chat-latest"
        self.llm_evaluation_temperature = 0.7
        self.next_turn_recipient = None
        self.faces_tables = {} # Removed: self._load_faces_tables()
        self.generation_lock = threading.Lock()
        self.is_generating_scenario = False
        self.start_signal_deferred = False
        self.is_turn_active = False
        self.pending_tts_finish = False
        self.generation_start_time = 0.0
        self.glass_castle_data = None
        self.behavioral_descriptors = None
        self.active_docs_request_id = None
        self.startup_check_triggered = False
        self.last_publish_metadata = None # Metadata for delayed synchronized terminal output
        
        # Watchdog for stuck generation
        self.create_timer(10.0, self._generation_watchdog)
        
        # --- Unique Fixed Voice Assignment ---
        self.assigned_voice_id = self._assign_voice_llm()
        self.get_logger().debug(f"[{self.role}] Assigned fixed voice: {self.assigned_voice_id}")

        # Leader startup sequence
        if self.role == self.family_config[0]:
            self.get_logger().info(f"[{self.role}] I am the leader. Preparing startup...")
            self.startup_timer = self.create_timer(5.0, self.initial_startup_check)
        self.last_triggered_step = -1
        self.last_evaluated_step = -1
        
        # Color Mapping
        self.COLOR_MAP = {
            "father": "\033[94m",  # Blue
            "mother": "\033[91m",  # Red
            "daughter": "\033[95m", # Magenta
            "son": "\033[92m",      # Green
            "reset": "\033[0m"
        }
        
        # Tone/Personality Mapping for Unbalanced Types


        self._load_config()
        self.user_intervention_lock_file = open(USER_INTERVENTION_LOCK_FILE, "a+")

        qos_tl = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.family_publisher = self.create_publisher(String, 'rfs_family_actions', 10)
        self.initial_scenario_pub = self.create_publisher(String, 'rfs_initial_scenario_generated', qos_tl)
        self.stt_resume_pub = self.create_publisher(String, 'rfs_stt_resume', 10)
        self.interrupt_tts_pub = self.create_publisher(String, 'rfs_interrupt_tts', 10)
        self.user_intervention_toio_move_pub = self.create_publisher(String, 'rfs_user_intervention_toio_move', 10)
        self.toio_move_script_pub = self.create_publisher(String, 'rfs_toio_move_script', 10)
        self.member_eval_pub = self.create_publisher(String, 'rfs_member_evaluation_results', 10)
        self.intervention_resolved_pub = self.create_publisher(String, 'rfs_intervention_resolved', 10)
        self.toio_move_finished_publisher = self.create_publisher(String, 'rfs_toio_move_finished', 10)
        self.trigger_eval_pub = self.create_publisher(String, 'rfs_trigger_evaluation', 10)
        self.generator_req_pub = self.create_publisher(String, 'rfs_generator_request', 10)
        self.member_eval_req_pub = self.create_publisher(String, 'rfs_member_eval_request', 10)
        self.behavior_req_pub = self.create_publisher(String, 'rfs_behavioral_info_request', 10)
        self.few_shot_req_pub = self.create_publisher(String, 'rfs_few_shot_request', 10)

        self.create_subscription(String, 'rfs_family_actions', self.message_callback, 10)
        self.create_subscription(String, 'rfs_user_intervention', self.user_intervention_callback, qos_tl)
        self.create_subscription(String, 'rfs_tts_finished', self.tts_finished_callback, 10)
        self.create_subscription(String, 'rfs_intervention_resolved', self.intervention_resolved_callback, 10)
        self.create_subscription(String, 'rfs_toio_position', self.toio_position_callback, 10)
        self.create_subscription(String, 'rfs_evaluation_complete', self.evaluation_complete_callback, 10)
        self.create_subscription(String, 'rfs_request_member_evaluation', self.request_member_evaluation_callback, 10)
        self.create_subscription(String, 'rfs_tts_initialization', self.tts_initialization_callback, qos_tl)
        self.create_subscription(String, 'rfs_toio_move_finished', self.move_finished_callback, 10)
        self.create_subscription(String, 'rfs_tts_status', self.tts_status_callback, 10)
        self.create_subscription(String, 'rfs_generator_results', self.generator_result_callback, 10)
        self.create_subscription(String, 'rfs_behavioral_info_results', self.behavioral_info_callback, 10)
        self.create_subscription(String, 'rfs_few_shot_results', self.few_shot_callback, 10)

        self.tts = TTSClient(node_name=self.role)
        self.get_logger().info(f"[{self.role}] Node started")
        
        # Immediate check for startup if leader
        if self.start_pending:
            self.create_timer(1.0, self._check_initialization)


    def _normalize_role_name(self, name: str) -> str:
        if not name: return ""
        n = name.lower().strip()
        # Mapping Japanese/English aliases to canonical roles
        lookup = {
            "父": "father", "お父さん": "father", "お父ちゃん": "father", "パパ": "father", "親父": "father",
            "母": "mother", "お母さん": "mother", "お母ちゃん": "mother", "ママ": "mother", "お袋": "mother",
            "娘": "daughter", "夏菜": "daughter", "お姉ちゃん": "daughter", "お姉さん": "daughter", "姉": "daughter",
            "息子": "son", "和也": "son", "お兄ちゃん": "son", "お兄さん": "son", "兄": "son", "弟": "son", "妹": "son",
            "祖父": "grandpa", "おじいちゃん": "grandpa", "おじいさん": "grandpa",
            "祖母": "grandma", "おばあちゃん": "grandma", "おばあさん": "grandma",
        }
        return lookup.get(n, n)

    def _load_config(self):
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.turns_per_step = config.get("turns_per_step", 10)
                    self.initial_coords = config.get("initial_coords", {"x": 8.0, "y": 8.0})
                    self.language = config.get("language", "en").lower()
                    self.llm_model = config.get("llm_model", "gpt-5.2-chat-latest")
                    self.llm_temperature = config.get("llm_temperature", 1.0)
                    self.llm_evaluation_model = config.get("llm_evaluation_model", "gpt-5.2-chat-latest")
                    self.llm_evaluation_temperature = config.get("llm_evaluation_temperature", 0.7)
        except Exception as e:
            self.get_logger().error(f"Config load error: {e}")

    def destroy_node(self):
        if hasattr(self, 'user_intervention_lock_file') and not self.user_intervention_lock_file.closed:
            self.user_intervention_lock_file.close()
        super().destroy_node()


    def behavioral_info_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("role") == self.role and data.get("request_id") == self.active_docs_request_id:
                self.behavioral_descriptors = data.get("behavioral_descriptors", "")
                self.get_logger().info(f"[{self.role}] Behavioral guidelines received (Req: {self.active_docs_request_id}).")
        except: pass

    def few_shot_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("role") == self.role and data.get("request_id") == self.active_docs_request_id:
                self.glass_castle_data = data.get("few_shot_context", "")
                self.get_logger().info(f"[{self.role}] Few-shot context received (Req: {self.active_docs_request_id}).")
        except: pass


    def tts_initialization_callback(self, msg: String):
        if msg.data.lower() == "tts_initialized":
            self.tts_ready = True
            self._check_initialization()

    def toio_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "toios_ready":
                self.toios_ready = True
                self.robot_positions = data.get("positions", {})
                self._check_initialization()
        except: pass

    def _check_initialization(self):
        if (self.chat_mode == 1 or self.move_enabled == 0):
            self.toios_ready = True
        if self.tts_ready and self.toios_ready and not self.initialization_done:
            self.initialization_done = True
            if self.start_pending:
                self.get_logger().debug(f"[{self.role}] Initialization complete. Starting designated first turn...")
                self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)
                self.initial_scenario_pub.publish(String(data="completed"))
                self.start_pending = False
                if self.role == self.family_config[0]: # If I am ALSO the leader, stop the startup fallback
                    if hasattr(self, 'startup_timer'): self.startup_timer.cancel()

    def initial_startup_check(self):
        # Stop timer if we already have history or turns
        turns = self._get_turn_count()
        if turns > 0:
            # self.get_logger().info(f"[{self.role}] Conversation started (turns={turns}). Stopping startup timer.")
            if hasattr(self, 'startup_timer'): self.startup_timer.cancel()
            return

        # STARTUP SYNCHRONIZATION: If a designated 'initial_role' is set and it's NOT us,
        # we give them 10 seconds head-start before the leader fallback fires.
        if self.initial_role and self.initial_role != self.role:
            if not hasattr(self, '_leader_wait_count'): self._leader_wait_count = 0
            self._leader_wait_count += 5 # Timer fires every 5s
            if self._leader_wait_count < 15:
                self.get_logger().info(f"[{self.role}] Leader waiting for initial speaker '{self.initial_role}' to start...")
                return

        if not os.path.exists(HISTORY_FILE) or os.path.getsize(HISTORY_FILE) < 10:
            if not self.is_generating_scenario and not self.startup_check_triggered:
                self.get_logger().info(f"[{self.role}] History empty. Starting first turn...")
                self.startup_check_triggered = True
                self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)
            else:
                self.get_logger().info(f"[{self.role}] History empty but already generating. Waiting...")
        else:
            # Check if conversation stalled (no recent dialogue)
            if turns == 0:
                if not self.is_generating_scenario and not self.startup_check_triggered:
                    self.get_logger().info(f"[{self.role}] Turn 0. Starting initial statement...")
                    self.startup_check_triggered = True
                    self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)
                else:
                    self.get_logger().info(f"[{self.role}] Turn 0 but already generating. Waiting...")

    def load_full_history(self) -> str:
        try:
            with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                lines = f.readlines()
            # Filter out clinical analysis and system updates to preserve the "fourth wall" for the character
            filtered = [l for l in lines if not (l.startswith("[THERAPIST_") or l.startswith("[SYSTEM_UPDATE"))]
            return "".join(filtered)
        except FileNotFoundError: return ""

    def _get_turn_count(self) -> int:
        count = 0
        if not os.path.exists(HISTORY_FILE): return 0
        try:
            with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                for line in f:
                    if "conversation" in line: count += 1
        except: pass
        return count

    def update_history(self, text: str, is_leader_response: bool = False):
        if text is None: return
        if self.is_scenario_generation_paused and not self.am_i_intervention_responder: return
        turns = self._get_turn_count()
        step_id = turns // self.turns_per_step
        
        write_text = text
        if not text.startswith("[SYSTEM_UPDATE]") and ("conversation" in text or "move" in text):
            write_text = f"S{step_id}_T{turns+1},{text}"

        prefix = "[LEADER_RESPONSE] " if is_leader_response else ""
        try:
            with open(HISTORY_FILE, "a", encoding="utf-8") as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                f.write(prefix + write_text + "\n")
                f.flush()
                try: os.fsync(f.fileno())
                except: pass
                fcntl.flock(f, fcntl.LOCK_UN)
        except Exception as e:
            self.get_logger().error(f"Failed to write history: {e}")

    def _generation_watchdog(self):
        with self.generation_lock:
            if self.is_generating_scenario:
                elapsed = time.time() - self.generation_start_time
                if elapsed > 45.0:
                    self.get_logger().error(f"[{self.role}] DETECTED STUCK GENERATION ({elapsed:.1f}s). Resetting lock.")
                    self.is_generating_scenario = False
                    if self.start_signal_deferred:
                        self.get_logger().info(f"[{self.role}] Re-triggering deferred generation...")
                        self.trigger_scenario_generation(force_publish=True)

    def trigger_scenario_generation(self, is_intervention: bool = False, intervention_text: str = None, 
                                   is_initial_statement: bool = False, force_publish: bool = False):
        if not self.initialization_done or (self.is_scenario_generation_paused and not is_intervention): return
        
        if self.waiting_for_evaluation:
            self.am_i_intervention_responder = is_intervention
            self.pending_intervention_text = intervention_text
            self.get_logger().info(f"[{self.role}] Deferred generation due to evaluation.")
            return

        with self.generation_lock:
            if self.is_generating_scenario:
                if is_initial_statement or force_publish:
                    self.get_logger().info(f"[{self.role}] Generation already in progress. Deferring finish trigger.")
                    self.start_signal_deferred = True
                return
            self.is_generating_scenario = True
            self.start_signal_deferred = False
            self.generation_start_time = time.time()
        
        def generation_task():
            try:
                # 1. Fetch current (x, y)
                trajectory = []
                if os.path.exists(TRAJECTORY_FILE):
                    try:
                        with open(TRAJECTORY_FILE, "r", encoding="utf-8") as f: trajectory = json.load(f)
                    except: pass
                
                if not trajectory:
                    x = self.initial_coords.get("x", 8.0); y = self.initial_coords.get("y", 8.0)
                else:
                    last = trajectory[-1]
                    x = last.get("target_x", last.get("x", 8.0))
                    y = last.get("target_y", last.get("y", 8.0))

                # 2. Request documents from DocumentProcessor
                self.behavioral_descriptors = None
                self.glass_castle_data = None
                self.active_docs_request_id = f"{self.role}_{self.step_count}_{time.time()}"
                self.get_logger().info(f"[{self.role}] Requesting clinical info (Req: {self.active_docs_request_id})")
                self.behavior_req_pub.publish(String(data=json.dumps({"x": x, "y": y, "role": self.role, "request_id": self.active_docs_request_id})))
                self.few_shot_req_pub.publish(String(data=json.dumps({"role": self.role, "request_id": self.active_docs_request_id})))
                
                # 3. Wait for results (increased timeout for robustness)
                wait_start = time.time()
                while (self.behavioral_descriptors is None or self.glass_castle_data is None) and (time.time() - wait_start < 5.0):
                    time.sleep(0.1)
                
                # Snapshot current values (even if defaults)
                bev_info = self.behavioral_descriptors if self.behavioral_descriptors is not None else ""
                few_shot_ref = self.glass_castle_data if self.glass_castle_data is not None else "No reference example available."
                
                if self.behavioral_descriptors is None or self.glass_castle_data is None:
                    self.get_logger().info(f"[{self.role}] Proceeding with default/empty clinical info (Timeout reached).")

                family_status = f"(Score: Cohesion={x:.1f}, Flexibility={y:.1f})\n"
                family_status += f"\n# FACES IV Behavioral Guidelines (STRICTLY FOLLOW THESE)\n{bev_info}"

                # 4. Prepare and Send to Generator
                try:
                    with open(VOICE_LIST_FILE, 'r', encoding='utf-8') as f: voice_list_content = f.read()
                    with open(CONFIG_FILE, 'r', encoding='utf-8') as f: config_data = json.load(f)
                    config_content = json.dumps(config_data, ensure_ascii=False, indent=2)
                except: config_content = ""; voice_list_content = ""; config_data = {}
                
                theme_anchor = config_data.get("theme", "Family Gathering")
                current_history = self.load_full_history()
                
                req_payload = {
                    "request_id": self.active_docs_request_id,
                    "role": self.role,
                    "language": self.language,
                    "family_config": self.family_config,
                    "target_user": self.target_user,
                    "assigned_voice_id": self.assigned_voice_id,
                    "family_status": family_status,
                    "theme_anchor": theme_anchor,
                    "voice_list_content": voice_list_content,
                    "config_content": config_content,
                    "current_history": current_history,
                    "few_shot_context": few_shot_ref,
                    "intervention_text": intervention_text,
                    "llm_model": self.llm_model,
                    "llm_temperature": self.llm_temperature,
                    "metadata": {
                        "is_intervention": is_intervention,
                        "is_initial_statement": is_initial_statement,
                        "force_publish": force_publish
                    }
                }
                
                self.generator_req_pub.publish(String(data=json.dumps(req_payload)))
                self.get_logger().info(f"[{self.role}] Dialogue request sent to generator (Req: {self.active_docs_request_id})")
                
            except Exception as e:
                self.get_logger().error(f"Error triggering generator: {e}")
                with self.generation_lock: self.is_generating_scenario = False

        threading.Thread(target=generation_task, daemon=True).start()

    def generator_result_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            role = data.get("role")
            if role != self.role: return

            scenario = data.get("scenario")
            metadata = data.get("metadata", {})
            is_intervention = metadata.get("is_intervention", False)
            is_initial_statement = metadata.get("is_initial_statement", False)
            force_publish = metadata.get("force_publish", False)

            if not scenario:
                self.get_logger().error(f"[{self.role}] Received empty scenario from generator.")
                with self.generation_lock: self.is_generating_scenario = False
                return

            # Post-generation handling
            if is_intervention:
                self.update_history(scenario, is_leader_response=True)
                reader = csv.reader(io.StringIO(scenario), skipinitialspace=True)
                row = next(reader)
                resp = row[0].lower()
                self.reset_intervention_state()
                if resp == self.role:
                    self.pending_scenario_conversation = scenario
                    self.publish_pending_scenario(from_leader_instruction=True)
                else:
                    self.get_logger().info(f"Relaying intervention response to {resp}")
                    t_msg = String()
                    t_msg.data = f"{self.role},{resp},resume_turn"
                    self.family_publisher.publish(t_msg)
            else:
                target_line = None
                lines = scenario.strip().split('\n')
                for line in lines:
                    line = line.strip()
                    if line.count(',') >= 2:
                        if (any(kw in line.lower() for kw in ['conversation', 'move', '会話', '動作', '移動']) or 
                            (line.startswith('S') and '_T' in line)):
                            target_line = line
                            break
                
                if not target_line:
                    # Loosen check: just look for a CSV line with enough columns
                    for line in lines:
                        line = line.strip()
                        if line.count(',') >= 5: # At least role, recipient, text, voice, style
                             target_line = line
                             break

                if not target_line:
                    self.get_logger().error(f"[{self.role}] Invalid CSV or Refusal detected from generator. RAW SCENARIO: {scenario[:200]}...")
                    with self.generation_lock: 
                        self.is_generating_scenario = False
                        self.startup_check_triggered = False # Reset so we can retry
                    return

                # Successfully parsed
                self.get_logger().debug(f"[{self.role}] Generation successful. Parsed line: {target_line}")
                
                if is_initial_statement:
                    self.initial_scenario_pub.publish(String(data=target_line))
                    # Removed redundant update_history here, it happens in publish_pending_scenario
                    # self.update_history(target_line)
                
                reader = csv.reader(io.StringIO(target_line), skipinitialspace=True)
                row = next(reader)
                if len(row) >= 3:
                     ctype = row[2].strip().lower()
                     if 'move' in ctype or '動作' in ctype or '移動' in ctype:
                         self.pending_scenario_move = target_line
                     else:
                         self.pending_scenario_conversation = target_line
                
                if is_initial_statement or force_publish:
                    if self.start_signal_deferred or force_publish:
                        self.get_logger().info(f"[{self.role}] Publishing initial/forced statement.")
                        self.publish_pending_scenario(force_publish=True)
                        self.start_signal_deferred = False
                    else:
                        self.get_logger().info(f"[{self.role}] Staging initial statement.")
                else:
                    # Pre-synthesis for regular turns if needed
                    if self.pending_scenario_conversation:
                        self.tts.speak(self.pending_scenario_conversation, delay=self.pending_delay)
                        self.audio_synthesis_requested = True

            with self.generation_lock:
                self.is_generating_scenario = False

        except Exception as e:
            self.get_logger().error(f"Error processing generator result: {e}")
            with self.generation_lock: self.is_generating_scenario = False

    def publish_pending_scenario(self, from_leader_instruction: bool = False, force_publish: bool = False):
        if self.waiting_for_evaluation and not from_leader_instruction: return
        
        with self.generation_lock:
            if self.is_generating_scenario and not force_publish:
                self.get_logger().debug(f"[{self.role}] Publish requested while still generating. Deferring start.")
                self.start_signal_deferred = True
                return

        if self.pending_scenario_conversation is None:
            if force_publish:
                self.get_logger().warn(f"[{self.role}] No pending conversation to publish. Wait for next trigger.")
                # self.trigger_scenario_generation(force_publish=True) # REMOVED: Prevent redundant retry loops
            return

        try:
            reader = csv.reader(io.StringIO(self.pending_scenario_conversation), skipinitialspace=True)
            parts = next(reader)
            if len(parts) > 1:
                # recipient_role is correctly stored in parts[1], not parts[2]
                recipient_role = parts[1].strip().lower()
                
                # Internal turn-tracking and history update
                turns = self._get_turn_count()
                step_idx = turns // self.turns_per_step
                color = self.COLOR_MAP.get(self.role, "")
                reset = self.COLOR_MAP.get("reset", "")
                
                # Store metadata for synchronized output (to be printed when playback starts)
                dialogue = parts[3] if len(parts) > 3 else "..."
                self.last_publish_metadata = {
                    "recipient": recipient_role,
                    "dialogue": dialogue,
                    "turn_label": f"S{step_idx}_T{turns+1}",
                    "color": color
                }

                # Prepare for relay
                next_target = self._normalize_role_name(recipient_role)
                OUTSIDERS = ['user', 'family', 'everyone', 'all', 'おじいちゃん', 'おばあちゃん', 'お父さん', 'お母さん', 'お兄さん', 'お姉さん', 'お兄ちゃん', 'お姉ちゃん', '弟', '妹', '祖父', '祖母', 'おじいさん', 'おばあさん', 'ゲスト', 'guest']
                if next_target not in self.family_config or next_target == self.role or next_target in OUTSIDERS:
                    others = [m for m in self.family_config if m != self.role]
                    if others: next_target = random.choice(others)
                    else: next_target = self.role
                self.pending_relay_recipient = next_target
                
                # Boundary check: if this is the last turn of the step, suppress pre-fetching the next speaker
                turns_after = turns + 1 # We are about to write this turn
                if turns_after > 0 and turns_after % self.turns_per_step == 0:
                    self.get_logger().debug(f"[{self.role}] Step boundary reached ({turns_after} turns). Evaluation will follow. Early relay suppressed.")
                    self.waiting_for_evaluation = True
                    self.pending_relay_recipient = None
                
                # Enforce fixed voice id in the scenario string for TTS
                if len(parts) > 5:
                    parts[4] = self.assigned_voice_id
                    parts[5] = self.assigned_voice_id
                    output = io.StringIO()
                    writer = csv.writer(output)
                    writer.writerow(parts)
                    self.pending_scenario_conversation = output.getvalue().strip()
        except Exception as e:
            self.get_logger().error(f"Error in publish_pending_scenario: {e}")

        self.update_history(self.pending_scenario_conversation)
        
        # Enforce fixed voice
        if not self.audio_synthesis_requested:
            # self.get_logger().info(f"[{self.role}] Requesting audio synthesis (foreground)...")
            self.tts.speak(self.pending_scenario_conversation, delay=self.pending_delay)
        else:
            # self.get_logger().info(f"[{self.role}] Audio already synthesizing in background. Proceeding to action.")
            pass

        self.family_publisher.publish(String(data=self.pending_scenario_conversation))
        
        # Turn is now officially started
        self.is_turn_active = True
        self.startup_check_triggered = False # Reset guard on success

        if self.pending_scenario_move:
            self.pending_move_command = self.pending_scenario_move
        
        self.pending_scenario_conversation = None
        self.pending_scenario_move = None

        # 2. RELAY PREPARATION (Early preparation for next speaker — ONE-AHEAD LIMIT)
        # Only queue the NEXT speaker's generation. No further ahead.
        if self.pending_relay_recipient and not self.waiting_for_evaluation and not self.next_generation_queued:
            next_target = self.pending_relay_recipient
            # self.get_logger().info(f"[{self.role}] Early relaying preparation to {next_target}")
            t_msg = String()
            t_msg.data = f"{self.role},{next_target},prepare_turn"
            self.family_publisher.publish(t_msg)
            self.next_turn_recipient = next_target # Store for later start_turn relay
            self.next_generation_queued = True  # Mark: next speaker is already queued
        
        if self.pending_tts_finish:
            # self.get_logger().info(f"[{self.role}] TTS finished early (flag=True). Triggering completion now.")
            self.pending_tts_finish = False
            self.tts_finished_callback(String(data=f"finished,{self.role}"))
        else:
            # self.get_logger().info(f"[{self.role}] No early TTS finish pending (flag=False). Waiting for normal signal.")
            pass

        self.pending_scenario_conversation = None
        self.pending_scenario_move = None
        self.audio_synthesis_requested = False
        if from_leader_instruction: self.stt_resume_pub.publish(String(data="resume"))



    def message_callback(self, msg: String):
        try:
            reader = csv.reader(io.StringIO(msg.data), skipinitialspace=True)
            parts = next(reader)
            if len(parts) < 3: return
            sender = parts[0].lower()
            target = parts[1].lower()
            cmd = parts[2].lower()
            
            # Decentralized autonomous model: trigger on prepare_turn, start_turn or resume_turn
            normalized_target = self._normalize_role_name(target)
            if normalized_target == self.role:
                if cmd == 'prepare_turn':
                    # self.get_logger().info(f"[{self.role}] Preparation signal received from {sender}. Generating scenario...")
                    self.trigger_scenario_generation(force_publish=False)
                elif cmd == 'start_turn' or cmd == 'resume_turn':
                    if self.pending_scenario_conversation is None:
                        # If no pre-generation happened (e.g. after evaluation), force it now
                        self.trigger_scenario_generation(force_publish=True)
                    else:
                        # self.get_logger().info(f"[{self.role}] Start signal '{cmd}' received from {sender}. Publishing scenario...")
                        self.publish_pending_scenario(from_leader_instruction=True, force_publish=True)
                elif cmd == 'conversation':
                    # Log but do not act on other's conversation messages
                    pass
        except Exception as e:
            # self.get_logger().error(f"Error in message_callback: {e}")
            pass

    def tts_status_callback(self, msg: String):
        try:
            # Format: start,role,text,muted
            parts = msg.data.split(',')
            if parts[0] == 'start':
                speaker = parts[1].lower()
                if speaker == self.role:
                    # self.get_logger().info(f"[{self.role}] I started playing. Checking for relay or evaluation...")
                    
                    # 1. EVALUATION CHECK
                    turns = self._get_turn_count()
                    if turns > 0 and turns % self.turns_per_step == 0:
                        step_idx = turns // self.turns_per_step
                        step_id = f"S{step_idx}"
                        if step_idx > self.last_triggered_step:
                            self.get_logger().debug(f"[{self.role}] Threshold reached. Global evaluation {step_id} will be triggered after audio/move.")
                            self.waiting_for_evaluation = True
                            self.last_triggered_step = step_idx
                            self.pending_eval_step_id = step_id
                            self.pending_relay_recipient = None # Stop relay if evaluation is pending
                    
                    # 2. SYNCHRONIZED TERMINAL OUTPUT
                    if self.last_publish_metadata:
                        m = self.last_publish_metadata
                        print(f"{m['color']}\n[{self.role} -> {m['recipient']}]")
                        print(f"{m['turn_label']}: {m['dialogue']}{self.COLOR_MAP['reset']}\n")
                        self.last_publish_metadata = None # Consume the metadata
                    
        except Exception as e:
            self.get_logger().error(f"Error in tts_status_callback: {e}")

    def tts_finished_callback(self, msg: String):
        try:
            # Format: finished,role
            parts = msg.data.split(',')
            sender = parts[1].lower()
            if sender != self.role: return
            
            if not self.is_turn_active:
                # self.get_logger().info(f"[{self.role}] Received early TTS finish signal. Deferring.")
                self.pending_tts_finish = True
                return
        except: return

        if self.pending_move_command and self.pending_move_command.lower() != 'none':
            self.get_logger().info(f"TTS finished. Executing move: {self.pending_move_command}")
            self.update_history(self.pending_move_command)
            self.toio_move_script_pub.publish(String(data=self.pending_move_command))
            self.pending_move_command = None
        else:
            # No move - trigger turn takeover immediately
            # self.get_logger().info("No move pending. Signaling turn completion.")
            pass
            self.pending_move_command = None
            finish_msg = String()
            finish_msg.data = json.dumps({"role": self.role, "status": "completed"})
            self.toio_move_finished_publisher.publish(finish_msg)

    def move_finished_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            finished_role = data.get("role", "").lower()
        except: return

        if finished_role != self.role:
            return

        # I AM THE ONE WHO FINISHED
        # If I have a deferred evaluation trigger, fire it now that my audio/move is done
        if self.pending_eval_step_id:
            self.get_logger().debug(f"[{self.role}] Turn complete. Triggering global evaluation for {self.pending_eval_step_id}.")
            self.trigger_eval_pub.publish(String(data=self.pending_eval_step_id))
            self.pending_eval_step_id = None
            return

        # If we are waiting for evaluation, do not relay
        if self.waiting_for_evaluation:
            self.get_logger().info(f"[{self.role}] Finished move, but evaluation is pending. Stopping relay.")
            return

        # Relay to next robot now that I am finished
        if self.next_turn_recipient:
            next_target = self.next_turn_recipient
            # self.get_logger().info(f"[{self.role}] Turn complete. Signaling GO to {next_target}")
            t_msg = String()
            t_msg.data = f"{self.role},{next_target},start_turn"
            self.family_publisher.publish(t_msg)
            self.next_turn_recipient = None
            self.pending_relay_recipient = None
        elif self.pending_relay_recipient:
            # Fallback if audio never started status
            next_target = self.pending_relay_recipient
            # self.get_logger().info(f"[{self.role}] Turn complete (fallback). Signaling GO to {next_target}")
            t_msg = String()
            t_msg.data = f"{self.role},{next_target},start_turn"
            self.family_publisher.publish(t_msg)
            self.pending_relay_recipient = None
        
        # Turn officially over — reset one-ahead queue guard
        self.is_turn_active = False
        self.next_generation_queued = False

    def user_intervention_callback(self, msg: String):
        text = msg.data.strip()
        if text == 'user_speech_started':
            self.is_scenario_generation_paused = True
            self.interrupt_tts_pub.publish(String(data="stop_all"))
        elif text.startswith('user:'):
            self.trigger_scenario_generation(is_intervention=True, intervention_text=text)

    def intervention_resolved_callback(self, msg: String):
        self.is_scenario_generation_paused = False

    def toio_position_callback(self, msg: String):
        try: self.robot_positions = json.loads(msg.data)
        except: pass


    def evaluation_complete_callback(self, msg: String):
        self.get_logger().debug(f"[{self.role}] Evaluation complete for {msg.data}. Resetting pending states for new step.")
        self.waiting_for_evaluation = False
        
        # Clear any pending scenario to force fresh generation for the new step guidelines
        self.pending_scenario_conversation = None
        self.pending_scenario_move = None
        self.audio_synthesis_requested = False

        # Leader coordinates the resumption of the conversation
        if self.role == self.family_config[0]:
            self.get_logger().debug(f"[{self.role}] Evaluation complete. Coordinating session resume...")
            # Brief delay to ensure all nodes have cleared their status
            time.sleep(1.0)
            
            # Ensure audio is unmuted before Turn 11 starts
            self.interrupt_tts_pub.publish(String(data="resume_all"))
            
            history = self.load_full_history()
            if not history:
                self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)
                return

            last_lines = history.strip().split('\n')
            last_conv_line = None
            for line in reversed(last_lines):
                if "," in line and "_T" in line:
                    last_conv_line = line
                    break

            if not last_conv_line:
                self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)
                return

            try:
                reader = csv.reader(io.StringIO(last_conv_line), skipinitialspace=True)
                parts = next(reader)
                if len(parts) < 3:
                     self.trigger_scenario_generation(force_publish=True); return
                
                # S0_T10,daughter,mother,conversation,... -> parts[1] is daughter, parts[2] is mother
                last_speaker = parts[1].strip().lower()
                last_recipient = parts[2].strip().lower()
                
                if last_recipient in self.family_config and last_recipient != last_speaker:
                    next_speaker = last_recipient
                else:
                    others = [m for m in self.family_config if m != last_speaker]
                    next_speaker = others[0] if others else self.role
                
                self.get_logger().debug(f"[{self.role}] Resuming: last speaker was '{last_speaker}', next is '{next_speaker}'.")
                
                if next_speaker == self.role:
                    self.trigger_scenario_generation(force_publish=True)
                else:
                    t_msg = String()
                    t_msg.data = f"{self.role},{next_speaker},resume_turn"
                    self.family_publisher.publish(t_msg)
            except Exception as e:
                self.get_logger().error(f"Error resuming conversation: {e}")
                self.trigger_scenario_generation(force_publish=True)

    def request_member_evaluation_callback(self, msg: String):
        step_id = msg.data
        try:
            step_idx = int(step_id.replace("S", ""))
        except: step_idx = 0

        if step_idx <= self.last_evaluated_step:
            return

        self.get_logger().info(f"[{self.role}] Subjective FACES IV evaluation requested for {step_id}. Forwarding to evaluator node.")
        self.last_evaluated_step = step_idx
        self.waiting_for_evaluation = True

        history = self.load_full_history()
        
        request = {
            "step_id": step_id,
            "role": self.role,
            "history": history,
            "llm_model": self.llm_evaluation_model,
            "llm_temperature": self.llm_evaluation_temperature
        }
        
        self.member_eval_req_pub.publish(String(data=json.dumps(request)))


    def reset_intervention_state(self):
        self.is_scenario_generation_paused = False
        self.intervention_resolved_pub.publish(String(data="resolved"))

    def _assign_voice_llm(self) -> str:
        try:
            with open(VOICE_LIST_FILE, 'r', encoding='utf-8') as f:
                v_list = json.load(f)
            
            # Robust gender detection (Supporting EN/JP and common family roles)
            gender = "female"
            male_keywords = ["father", "son", "brother", "grandpa", "grandfather", "uncle", "boy", "man", "male", 
                             "父", "父さん", "お父さん", "パパ", "息子", "兄", "弟", "おじいさん", "おじいちゃん", "祖父", "叔父", "伯父", "男"]
            if any(k in self.role.lower() for k in male_keywords):
                gender = "male"
            
            # Additional heuristic: roles containing 'daughter', 'mother', 'girl', etc are always female
            female_keywords = ["mother", "daughter", "sister", "grandma", "grandmother", "aunt", "girl", "woman", "female",
                               "母", "母さん", "お母さん", "ママ", "娘", "姉", "妹", "おばあさん", "おばあちゃん", "祖母", "叔母", "伯母", "女"]
            if gender == "male" and any(k in self.role.lower() for k in female_keywords):
                # If there's a conflict, default to female (usually safer for ambiguous roles)
                gender = "female"
            
            candidates = [v for v in v_list if v.get("gender", "").lower() == gender]
            if not candidates: candidates = v_list
            
            self.get_logger().debug(f"[{self.role}] Detected gender: {gender}. Filtering {len(candidates)} candidates.")


            # Use LLM to pick the best match for the role and theme
            prompt = f"Role: {self.role}\nTheme: {self.theme}\nGender Requirement: {gender}\nAvailable Voices: {[{'name':v['name'], 'overview':v['overview']} for v in candidates]}\n\nPick the most suitable voice name for this role from the list above. You MUST pick one of the names from 'Available Voices'. Output ONLY the name."
            response = client.chat.completions.create(
                model="gpt-5.2-chat-latest",
                messages=[{"role": "user", "content": prompt}],
                max_completion_tokens=100
            )
            v_name = response.choices[0].message.content.strip().replace('"', '').replace("'", "")
            
            # Strict validation
            found_voice = next((v for v in candidates if v['name'].lower() == v_name.lower()), None)
            if found_voice:
                return found_voice['name']
            
            # Fallback to first suitable gendered voice
            self.get_logger().warn(f"[{self.role}] LLM provided invalid voice name '{v_name}'. Falling back.")
            return candidates[0]["name"]
        except Exception as e:
            self.get_logger().error(f"Voice assignment error: {e}")
            return "Kore"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", type=str, required=True)
    parser.add_argument("--initiate", action="store_true")
    args, _ = parser.parse_known_args()
    rclpy.init()
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            config = json.load(f)
        family_config = config.get("family_config", ["father", "mother", "daughter"])
        theme = config.get("theme", "Family Dinner")
        chat_mode = config.get("chat_mode", 0)
        target_user = config.get("target_user", "User")
        move_enabled = config.get("toio_move", 1)
    except:
        family_config = ["father", "mother", "daughter"]
        theme = "Family Dinner"; chat_mode = 0; target_user = "User"; move_enabled = 0

    node = RFSFamilyMember(role=args.role, theme=theme, chat_mode=chat_mode, target_user=target_user, move=move_enabled, family_config=family_config, initial_role=args.role if args.initiate else "")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
