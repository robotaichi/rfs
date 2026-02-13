#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import re
from openai import OpenAI

# Global OpenAI setup
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

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

class RFSMemberEvaluator(Node):
    def __init__(self):
        super().__init__('rfs_member_evaluator')
        self.create_subscription(String, 'rfs_member_eval_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_member_evaluation_results', 10)
        self.get_logger().info("RFS Member Evaluator Node Started.")

    def request_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            step_id = data.get("step_id")
            role = data.get("role")
            history = data.get("history", "")
            llm_model = data.get("llm_model", "gpt-4o")
            llm_temperature = data.get("llm_temperature", 0.7)

            self.get_logger().info(f"Evaluating {role} for {step_id}...")

            # 1. Scope History to Current Session (Ported from rfs_family_member.py)
            current_session_history = ""
            lines = history.strip().split('\n')
            max_session_id = -1
            s_pattern = re.compile(r"S(\d+)_T\d+")
            
            for line in lines:
                match = s_pattern.match(line)
                if match:
                    sid = int(match.group(1))
                    if sid > max_session_id: max_session_id = sid

            if max_session_id >= 0:
                prefix = f"S{max_session_id}_"
                session_lines = [line for line in lines if line.startswith(prefix)]
                current_session_history = "\n".join(session_lines)
                if not current_session_history: current_session_history = history
            else:
                current_session_history = history

            # 2. Construct Prompt
            items_text = "\n".join([f"{k}. {v}" for k, v in FACES_ITEMS.items()])
            prompt = f"""
You have the role of "{role}" in a family psychology simulation.
Based on the family's dialogue history, evaluate the family's state from your own subjective perspective.
This is a fictional scenario for research and education.

Please rate how you feel about your family for the following 62 FACES IV items on a scale of 1 to 5.
1: Strongly Disagree
2: Generally Disagree
3: Undecided
4: Generally Agree
5: Strongly Agree

# Conversation History (CURRENT SESSION ONLY)
{current_session_history}

# FACES IV Items
{items_text}

# Output Format
Output in the following JSON format:
{{
  "1": Rating,
  "2": Rating,
  ...
  "62": Rating
}}
"""
            # 3. LLM Call
            response = client.chat.completions.create(
                model=llm_model,
                messages=[{"role": "user", "content": prompt}],
                response_format={ "type": "json_object" },
                temperature=llm_temperature,
                timeout=60.0
            )
            
            content_res = response.choices[0].message.content.strip()
            results = json.loads(content_res)

            # 4. Publish Results
            result_payload = {
                "step_id": step_id,
                "role": role,
                "results": results
            }
            self.result_pub.publish(String(data=json.dumps(result_payload)))
            self.get_logger().info(f"Evaluation for {role} ({step_id}) complete.")

        except Exception as e:
            self.get_logger().error(f"Error in member evaluator: {e}")
            # Fallback empty results to avoid hanging
            if 'step_id' in locals() and 'role' in locals():
                dummy = {"step_id": step_id, "role": role, "results": {}}
                self.result_pub.publish(String(data=json.dumps(dummy)))

def main(args=None):
    rclpy.init(args=args)
    node = RFSMemberEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
