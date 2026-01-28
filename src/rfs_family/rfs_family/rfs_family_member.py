#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String, Bool
import sys
import os
import json
import openai
import time
import argparse
import random
import csv
import io
import fcntl
import threading
from rfs_interfaces.srv import TTSService
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from ament_index_python.packages import get_package_share_directory

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
SINGLE_MEMBER_VOICE = {
    "id": "23",
    "name": "WhiteCUL",
    "style": "Normal",
    "reason": "To match soft and neutral communication."
}

FACES_ITEMS = {
    1: "Family members are involved in each others lives.",
    2: "Our family tries new ways of dealing with problems.",
    3: "We get along better with people outside our family than inside.",
    4: "We spend too much time together.",
    5: "There are strict consequences for breaking the rules in our family.",
    6: "We never seem to get organized in our family.",
    7: "Family members feel very close to each other.",
    8: "Parents equally share leadership in our family.",
    9: "Family members seem to avoid contact with each other when at home.",
    10: "Family members feel pressured to spend most free time together.",
    11: "There are clear consequences when a family member does something wrong.",
    12: "It is hard to know who the leader is in our family.",
    13: "Family members are supportive of each other during difficult times.",
    14: "Discipline is fair in our family.",
    15: "Family members know very little about the friends of other family members.",
    16: "Family members are too dependent on each other.",
    17: "Our family has a rule for almost every possible situation.",
    18: "Things do not get done in our family.",
    19: "Family members consult other family members on important decisions.",
    20: "My family is able to adjust to change when necessary.",
    21: "Family members are on their own when there is a problem to be solved.",
    22: "Family members have little need for friends outside the family.",
    23: "Our family is highly organized.",
    24: "It is unclear who is responsible for things (chores, activities) in our family.",
    25: "Family members like to spend some of their free time with each other.",
    26: "We shift household responsibilities from person to person.",
    27: "Our family seldom does things together.",
    28: "We feel too connected to each other.",
    29: "Our family becomes frustrated when there is a change in our plans or routines.",
    30: "There is no leadership in our family.",
    31: "Although family members have individual interests, they still participant in family activities.",
    32: "We have clear rules and roles in our family.",
    33: "Family members seldom depend on each other.",
    34: "We resent family members doing things outside the family.",
    35: "It is important to follow the rules in our family.",
    36: "Our family has a hard time keeping track of who does various household tasks.",
    37: "Our family has a good balance of separateness and closeness.",
    38: "When problems arise, we compromise.",
    39: "Family members mainly operate independently.",
    40: "Family members feel guilty if they want to spend time away from the family.",
    41: "Once a decision is made, it is very difficult to modify that decision.",
    42: "Our family feels hectic and disorganized.",
    43: "Family members are satisfied with how they communicate with each other.",
    44: "Family members are very good listeners.",
    45: "Family members express affection to each other.",
    46: "Family members are able to ask each other for what they want.",
    47: "Family members can calmly discuss problems with each other.",
    48: "Family members discuss their ideas and beliefs with each other.",
    49: "When family members ask questions of each other, they get honest answers.",
    50: "Family members try to understand each other’s feelings.",
    51: "When angry, family members seldom say negative things about each other.",
    52: "Family members express their true feelings to each other.",
    53: "The degree of closeness between family members.",
    54: "Your family’s ability to cope with stress.",
    55: "Your family’s ability to be flexible.",
    56: "Your family’s ability to share positive experiences.",
    57: "The quality of communication between family members.",
    58: "Your family’s ability to resolve conflicts.",
    59: "The amount of time you spend together as a family.",
    60: "The way problems are discussed.",
    61: "The fairness of criticism in your family.",
    62: "Family members concern for each other."
}

# OpenAI Client
if not os.environ.get("OPENAI_API_KEY"):
    print("Please set the OPENAI_API_KEY environment variable.")
    sys.exit(1)
client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

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
        
        self.step_count = 0
        self.waiting_for_evaluation = False
        self.turns_per_step = 10
        self.initial_coords = {"x": 8.0, "y": 8.0}
        self.pending_intervention_text = None
        self.pending_relay_recipient = None
        self.pending_eval_step_id = None # Track if we are the one to trigger evaluation
        self.language = "en"
        self.llm_model = "gpt-4o"
        self.llm_temperature = 1.0
        self.llm_evaluation_model = "gpt-4o"
        self.llm_evaluation_temperature = 0.7
        
        # --- Unique Fixed Voice Assignment ---
        self.assigned_voice_id = "Kore"
        try:
            with open(VOICE_LIST_FILE, 'r', encoding='utf-8') as f:
                v_list = json.load(f)
                role_idx = self.family_config.index(self.role) if self.role in self.family_config else 0
                # Assign unique voice based on index
                self.assigned_voice_id = v_list[role_idx % len(v_list)]["name"]
        except Exception as e:
            self.get_logger().error(f"Voice assignment error: {e}")
        self.get_logger().info(f"[{self.role}] Assigned fixed voice: {self.assigned_voice_id}")

        # Leader startup sequence
        if self.role == self.family_config[0]:
            self.get_logger().info(f"[{self.role}] I am the leader. Preparing startup...")
            self.create_timer(5.0, self.initial_startup_check)
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

        self.tts = TTSClient(node_name=self.role)
        self.get_logger().info(f"[{self.role}] Node started")

    def _load_config(self):
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.turns_per_step = config.get("turns_per_step", 10)
                    self.initial_coords = config.get("initial_coords", {"x": 8.0, "y": 8.0})
                    self.language = config.get("language", "en").lower()
                    self.llm_model = config.get("llm_model", "gpt-4o")
                    self.llm_temperature = config.get("llm_temperature", 1.0)
                    self.llm_evaluation_model = config.get("llm_evaluation_model", "gpt-4o")
                    self.llm_evaluation_temperature = config.get("llm_evaluation_temperature", 0.7)
        except Exception as e:
            self.get_logger().error(f"Config load error: {e}")

    def destroy_node(self):
        if hasattr(self, 'user_intervention_lock_file') and not self.user_intervention_lock_file.closed:
            self.user_intervention_lock_file.close()
        super().destroy_node()

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
        if (self.chat_mode == 1 or self.move_enabled == 0) and not self.toios_ready:
            self.toios_ready = True
        if self.tts_ready and self.toios_ready and not self.initialization_done:
            self.initialization_done = True
            if self.start_pending:
                self.trigger_scenario_generation(is_initial_statement=True)
                self.initial_scenario_pub.publish(String(data="completed"))

    def initial_startup_check(self):
        # Check if history is empty to start S0_T1
        if not os.path.exists(HISTORY_FILE) or os.path.getsize(HISTORY_FILE) < 10:
            self.get_logger().info(f"[{self.role}] History empty. Starting first turn...")
            self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)
        else:
            # Check if conversation stalled (no recent dialogue)
            turns = self._get_turn_count()
            if turns == 0:
                self.get_logger().info(f"[{self.role}] Turn 0. Starting initial statement...")
                self.trigger_scenario_generation(is_initial_statement=True, force_publish=True)

    def load_full_history(self) -> str:
        try:
            with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                return f.read()
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
        with open(HISTORY_FILE, "a", encoding="utf-8") as f:
            f.write(prefix + write_text + "\n")

    def trigger_scenario_generation(self, is_intervention: bool = False, intervention_text: str = None, 
                                   is_initial_statement: bool = False, force_publish: bool = False):
        if not self.initialization_done or (self.is_scenario_generation_paused and not is_intervention): return
        
        if self.waiting_for_evaluation:
            self.am_i_intervention_responder = is_intervention
            self.pending_intervention_text = intervention_text
            self.get_logger().info(f"[{self.role}] Deferred generation due to evaluation.")
            return
        scenario = self.generate_scenario(is_initial_statement=is_initial_statement, intervention_text=intervention_text)
        if not scenario: return
        if is_intervention:
            self.update_history(scenario, is_leader_response=True)
            try:
                reader = csv.reader(io.StringIO(scenario), skipinitialspace=True)
                row = next(reader)
                resp = row[0].lower()
                self.reset_intervention_state()
                if resp == self.role:
                    self.pending_scenario_conversation = scenario
                    self.publish_pending_scenario(from_leader_instruction=True)
                else:
                    self.get_logger().info(f"Relaying to {resp}")
            except: pass
        else:
            lines = scenario.strip().split('\n')
            for line in lines:
                try:
                    reader = csv.reader(io.StringIO(line), skipinitialspace=True)
                    parts = next(reader)
                    if len(parts) > 2:
                        type_tag = parts[2].lower()
                        if 'conversation' in type_tag: self.pending_scenario_conversation = line
                        elif 'move' in type_tag: self.pending_scenario_move = line
                except:
                    if 'conversation' in line: self.pending_scenario_conversation = line
                    elif 'move' in line: self.pending_scenario_move = line
            if is_initial_statement or force_publish: self.publish_pending_scenario(force_publish=True)

    def publish_pending_scenario(self, from_leader_instruction: bool = False, force_publish: bool = False):
        if self.waiting_for_evaluation and not from_leader_instruction: return
        
        if self.pending_scenario_conversation is None:
            self.get_logger().warn(f"[{self.role}] No pending conversation to publish.")
            return

        # Parse recipient for turn-taking
        recipient_role = "family"
        try:
            reader = csv.reader(io.StringIO(self.pending_scenario_conversation), skipinitialspace=True)
            parts = next(reader)
            if len(parts) > 1:
                recipient_role = parts[1].lower()
                
                # Color-coded display
                color = self.COLOR_MAP.get(self.role, "\033[96m")
                reset = self.COLOR_MAP["reset"]
                turns = self._get_turn_count()
                step_idx = turns // self.turns_per_step
                
                print(f"{color}\n[{self.role} -> {recipient_role}]")
                dialogue = parts[3] if len(parts) > 3 else "..."
                rationale = parts[7] if len(parts) > 7 else "No rationale provided."
                print(f"S{step_idx}_T{turns+1}: {dialogue}")
                print(f"Rationale: {rationale}\n{reset}")

                # Prepare for relay
                next_target = recipient_role
                OUTSIDERS = ['user', 'family', 'everyone', 'all', 'おじいちゃん', 'おばあちゃん', 'お父さん', 'お母さん', 'お兄さん', 'お姉さん', 'お兄ちゃん', 'お姉ちゃん', '弟', '妹', '祖父', '祖母', 'おじいさん', 'おばあさん', 'ゲスト', 'guest']
                if next_target not in self.family_config or next_target == self.role or next_target in OUTSIDERS:
                    others = [m for m in self.family_config if m != self.role]
                    if others: next_target = random.choice(others)
                    else: next_target = self.role
                self.pending_relay_recipient = next_target
                
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
        self.tts.speak(self.pending_scenario_conversation, delay=self.pending_delay)
        self.family_publisher.publish(String(data=self.pending_scenario_conversation))
        
        if self.pending_scenario_move:
            self.pending_move_command = self.pending_scenario_move

        self.pending_scenario_conversation = None
        self.pending_scenario_move = None
        if from_leader_instruction: self.stt_resume_pub.publish(String(data="resume"))

    def _get_family_type_info(self):
        try:
            if not os.path.exists(TRAJECTORY_FILE):
                return "Unknown", "Unknown", (50.0, 50.0), "No evaluation data"
            with open(TRAJECTORY_FILE, "r", encoding="utf-8") as f:
                trajectory = json.load(f)
            def get_coh_label(val):
                if val <= 15: return "Disengaged", "Family bonds are extremely weak, individuals are too independent."
                if val <= 35: return "Somewhat Connected", "Some bonds exist, but individual time is prioritized."
                if val <= 65: return "Connected", "Healthy balance between bonding and individual independence."
                if val <= 85: return "Very Connected", "Very strong bonds and shared time, while respecting individual autonomy."
                return "Enmeshed", "Overly enmeshed family; individual autonomy is lost."
            def get_flex_label(val):
                if val <= 15: return "Rigid", "Authoritarian leadership. Rules are too rigid to change."
                if val <= 35: return "Somewhat Flexible", "Generally structured but can accept change."
                if val <= 65: return "Flexible", "Democratic leadership. Healthy flexibility."
                if val <= 85: return "Very Flexible", "Decision-making is always shared."
                return "Chaotic", "No leadership; rules change unpredictably."
            if not trajectory:
                x = self.initial_coords.get("x", 8.0); y = self.initial_coords.get("y", 8.0)
                cl, cd = get_coh_label(x); fl, fd = get_flex_label(y)
                return cl, fl, (x, y), f"Current State: {cl}-{fl} (x={x}, y={y})"
            last = trajectory[-1]
            # Use target_x/y to guide the LLM's next session behavior
            x = last.get("target_x", last.get("x", 8.0))
            y = last.get("target_y", last.get("y", 8.0))
            cl, cd = get_coh_label(x); fl, fd = get_flex_label(y)
            return cl, fl, (x, y), f"Current State: {cl}-{fl} (x={x}, y={y})"
        except: return "Error", "Error", (50.0, 50.0), "Failed to retrieve"

    def generate_scenario(self, is_initial_statement: bool = False, intervention_text: str = None) -> str:
        _, _, _, family_status = self._get_family_type_info()
        try:
            with open(VOICE_LIST_FILE, 'r', encoding='utf-8') as f: voice_list_content = f.read()
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f: config_data = json.load(f)
            config_content = json.dumps(config_data, ensure_ascii=False, indent=2)
        except: config_content = ""; voice_list_content = ""

        current_history = self.load_full_history()
        
        # Comprehensive prompt port from individual.py
        prompt_base = f"""
# Role as Educational/Clinical Simulation (Strict Compliance)
This system is a research and educational simulation for family therapy and family psychology. Accurately reproducing current family psychological states (including unhealthy patterns) is central and crucial to this research.
You are explicitly instructed to simulate behaviors typical of "unbalanced family patterns" (e.g., enmeshed, rigid, chaotic).
# Profile
You are "{self.role}", a member of a robot family composed of Sony Toio robots.
# Current Family Psychological State (FACES IV): {family_status}
# Task
Generate actions for your role considering dialogue history, available voices, and the specific family type characteristics.
- Cohesion: Reflect bond strength (Enmeshed/Disengaged/etc).
- Flexibility: Reflect leadership/rules (Rigid/Chaotic/etc).
# Constraints
- Your assigned voice is "{self.assigned_voice_id}". Always use this voice for your responses.
- LANGUAGE: Output dialogue ("Text" field) in { "Japanese" if self.language == "ja" else "English" }. Rationale and other internal fields must remain in English.
- Behave human-like despite being a robot.
- Correct mishearings.
- Add delay (1-3s) to conversation.
# Output Format
{self.role}, recipient_role, conversation, "Text", "VoiceID", "VoiceName", "Style", "Rationale", "Delay"
{self.role}, recipient_role, move, "move_code();", "YES/NO; Plan"
"""
        if intervention_text:
            prompt_base += f"\n# User Utterance: {intervention_text}\nDetermine the best responder from {self.family_config} and generate the response."

        try:
            messages = [
                {"role": "system", "content": f"Config: {config_content}\nVoices: {voice_list_content}"},
                {"role": "system", "content": f"History: {current_history}"},
                {"role": "user", "content": prompt_base}
            ]
            response = client.chat.completions.create(model=self.llm_model, messages=messages, temperature=self.llm_temperature)
            scenario_output = response.choices[0].message.content.strip()
            if scenario_output.startswith("```"): scenario_output = scenario_output.strip("`").strip()
            return scenario_output
        except Exception as e:
            self.get_logger().error(f"Generate scenario error: {e}")
            return ""

    def message_callback(self, msg: String):
        try:
            reader = csv.reader(io.StringIO(msg.data), skipinitialspace=True)
            parts = next(reader)
            cmd = parts[2].lower()
            target = parts[1].lower()
            
            # Decentralized autonomous model: trigger on prepare_turn or resume_turn
            if target == self.role:
                if cmd == 'prepare_turn' or cmd == 'resume_turn':
                    force = (cmd == 'resume_turn')
                    self.get_logger().info(f"[{self.role}] Turn signal '{cmd}' received. Generating scenario...")
                    self.trigger_scenario_generation(force_publish=force)
        except: pass

    def tts_status_callback(self, msg: String):
        try:
            # Format: start,role,text,muted
            parts = msg.data.split(',')
            if parts[0] == 'start':
                speaker = parts[1].lower()
                if speaker == self.role:
                    self.get_logger().info(f"[{self.role}] I started playing. Checking for relay or evaluation...")
                    
                    # 1. EVALUATION CHECK
                    turns = self._get_turn_count()
                    if turns > 0 and turns % self.turns_per_step == 0:
                        step_idx = turns // self.turns_per_step
                        step_id = f"S{step_idx}"
                        if step_idx > self.last_triggered_step:
                            self.get_logger().info(f"[{self.role}] Threshold reached. Global evaluation {step_id} will be triggered after audio/move.")
                            self.waiting_for_evaluation = True
                            self.last_triggered_step = step_idx
                            self.pending_eval_step_id = step_id
                            self.pending_relay_recipient = None # Stop relay if evaluation is pending
                    
                    # 2. EARLY RELAY (Only if NOT evaluation period)
                    if not self.waiting_for_evaluation and self.pending_relay_recipient:
                        next_target = self.pending_relay_recipient
                        self.get_logger().info(f"[{self.role}] Relaying preparation to {next_target}")
                        t_msg = String()
                        t_msg.data = f"{self.role},{next_target},prepare_turn"
                        self.family_publisher.publish(t_msg)
                        self.pending_relay_recipient = None
        except Exception as e:
            self.get_logger().error(f"Error in tts_status_callback: {e}")

    def tts_finished_callback(self, msg: String):
        try:
            # Format: finished,role
            parts = msg.data.split(',')
            sender = parts[1].lower()
            if sender != self.role: return
        except: return

        if self.pending_move_command and self.pending_move_command.lower() != 'none':
            self.get_logger().info(f"TTS finished. Executing move: {self.pending_move_command}")
            self.update_history(self.pending_move_command)
            self.toio_move_script_pub.publish(String(data=self.pending_move_command))
            self.pending_move_command = None
        else:
            # No move - trigger turn takeover immediately
            self.get_logger().info("No move pending. Signaling turn completion.")
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
            self.get_logger().info(f"[{self.role}] Turn complete. Triggering global evaluation for {self.pending_eval_step_id}.")
            self.trigger_eval_pub.publish(String(data=self.pending_eval_step_id))
            self.pending_eval_step_id = None
            return

        # If we are waiting for evaluation, do not relay
        if self.waiting_for_evaluation:
            self.get_logger().info(f"[{self.role}] Finished move, but evaluation is pending. Stopping relay.")
            return

        # Relay to next robot now that I am finished
        if self.pending_relay_recipient:
            next_target = self.pending_relay_recipient
            self.get_logger().info(f"[{self.role}] Move finished. Relaying turn to {next_target}")
            t_msg = String()
            t_msg.data = f"{self.role},{next_target},prepare_turn"
            self.family_publisher.publish(t_msg)
            self.pending_relay_recipient = None

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
        self.get_logger().info(f"[{self.role}] Evaluation complete for {msg.data}.")
        self.waiting_for_evaluation = False
        
        # Leader coordinates the resumption of the conversation
        if self.role == self.family_config[0]:
            self.get_logger().info(f"[{self.role}] Evaluation complete. Coordinating session resume...")
            
            history = self.load_full_history()
            if not history:
                self.trigger_scenario_generation(force_publish=True)
                return

            last_lines = history.strip().split('\n')
            last_conv_line = None
            for line in reversed(last_lines):
                if "," in line and "_T" in line:
                    last_conv_line = line
                    break

            if not last_conv_line:
                self.trigger_scenario_generation(force_publish=True)
                return

            try:
                reader = csv.reader(io.StringIO(last_conv_line), skipinitialspace=True)
                parts = next(reader)
                if len(parts) < 3:
                     self.trigger_scenario_generation(force_publish=True); return
                
                last_speaker = parts[1].strip().lower()
                last_recipient = parts[2].strip().lower()
                
                if last_recipient in self.family_config and last_recipient != last_speaker:
                    next_speaker = last_recipient
                else:
                    others = [m for m in self.family_config if m != last_speaker]
                    next_speaker = others[0] if others else self.role
                
                self.get_logger().info(f"[{self.role}] Resuming: last speaker was '{last_speaker}', relaying to '{next_speaker}'.")
                
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
            self.get_logger().info(f"[{self.role}] Already evaluated {step_id}. Skipping.")
            return

        self.get_logger().info(f"[{self.role}] Starting subjective FACES IV evaluation for {step_id}...")
        self.last_evaluated_step = step_idx
        self.waiting_for_evaluation = True # Ensure flag is set if we got request without trigger

        # Running LLM evaluation (blocking)
        results = self.perform_faces_evaluation()
        
        if results:
            response = {
                "step_id": step_id,
                "role": self.role,
                "results": results
            }
            res_msg = String()
            res_msg.data = json.dumps(response)
            self.member_eval_pub.publish(res_msg)
            self.get_logger().info(f"[{self.role}] Evaluation results sent for {step_id}")

    def perform_faces_evaluation(self):
        history = self.load_full_history()
        if not history:
            self.get_logger().warn(f"[{self.role}] No history found for evaluation.")
            return None

        items_text = "\n".join([f"{k}. {v}" for k, v in FACES_ITEMS.items()])
        
        prompt = f"""
You have the role of "{self.role}" in a family psychology simulation.
Based on the family's dialogue history, evaluate the family's state from your own subjective perspective.
This is a fictional scenario for research and education.

Please rate how you feel about your family for the following 62 FACES IV items on a scale of 1 to 5.
1: Strongly Disagree
2: Generally Disagree
3: Undecided
4: Generally Agree
5: Strongly Agree

# Conversation History
{history}

# FACES IV Items
{items_text}

# Output Format
Output in the following JSON format:
{{
  "1": {{ "rating": Rating, "reason": "Reason (short sentence)" }},
  "2": {{ "rating": Rating, "reason": "Reason" }},
  ...
  "62": {{ "rating": Rating, "reason": "Reason" }}
}}
"""
        self.get_logger().info(f"[{self.role}] Requesting subjective evaluation from OpenAI...")
        try:
            response = client.chat.completions.create(
                model=self.llm_evaluation_model,
                messages=[{"role": "user", "content": prompt}],
                response_format={ "type": "json_object" },
                temperature=self.llm_evaluation_temperature
            )
            content_res = response.choices[0].message.content.strip()
            return json.loads(content_res)
        except Exception as e:
            self.get_logger().error(f"Error in perform_faces_evaluation for {self.role}: {e}")
            return None

    def reset_intervention_state(self):
        self.is_scenario_generation_paused = False
        self.intervention_resolved_pub.publish(String(data="resolved"))

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
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
