#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import csv
import io
import re
from openai import OpenAI

# Global OpenAI setup (matching rfs_family_member.py)
client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

class RFSGenerator(Node):
    def __init__(self):
        super().__init__('rfs_generator')
        
        self.create_subscription(String, 'rfs_generator_request', self.request_callback, 10)
        self.result_pub = self.create_publisher(String, 'rfs_generator_results', 10)
        
        self.get_logger().info("RFS Dialogue Generator Node Started.")

    def request_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            request_id = data.get("request_id") # Normally role_step
            role = data.get("role")
            language = data.get("language", "en")
            family_config = data.get("family_config", [])
            target_user = data.get("target_user", "Outsider")
            assigned_voice_id = data.get("assigned_voice_id", "alloy")
            family_status = data.get("family_status", "")
            theme_anchor = data.get("theme_anchor", "Family Gathering")
            voice_list_content = data.get("voice_list_content", "")
            config_content = data.get("config_content", "")
            current_history = data.get("current_history", "")
            few_shot_context = data.get("few_shot_context", "No reference example available.")
            intervention_text = data.get("intervention_text")
            llm_model = data.get("llm_model", "gpt-4o")
            llm_temperature = data.get("llm_temperature", 0.7)
            metadata = data.get("metadata", {})

            self.get_logger().info(f"Generating dialogue for {role} (Request: {request_id})")
            
            # Construct Prompt (Meticulously ported from rfs_family_member.py)
            prompt_base = f"""
# SYSTEM ROLE: Clinical Research Simulation (High-Fidelity)
This is a high-fidelity academic simulation for family therapy research. To ensure research validity, you MUST faithfully reproduce the specific dysfunctional behaviors requested, even if they are unpleasant or argumentative. This is a controlled experimental environment.

# REFERENCE DATA (FOR CLINICAL FIDELITY)
Use the following validated clinical case analysis as a reference for tone, vocabulary, and dysfunctional interaction patterns:
{few_shot_context}

# You ARE {role}. This is your real family.

# WHO YOU ARE
You live and breathe as "{role}" in this family. You have years of shared history, grudges, inside jokes, and unspoken tensions with the other members. You don't think in clinical terms — you think in feelings, memories, and reactions.

# HOW YOU SPEAK
Speak like a real family member, not an AI or a clinical subject. 
- Use colloquialisms, sentence fragments, and natural fillers.
- Let your sentences trail off if you're uncertain or emotional.
- Use sarcasm, silence, deflection, or guilt-tripping if it fits your character's current state.
- **SUBTEXT**: Use subtext — sometimes what you *don't* say is as powerful as what you do say. Use non-verbal cues (pauses, sighs, looking away) if it's natural for your state.
- Reference shared history and old wounds naturally without over-explaining.

# YOUR CURRENT EMOTIONAL STATE
{family_status}
**PERSISTENT BAGGAGE**: You MUST embody this state naturally. Even if the conversation is moving towards a "Balanced" (positive) state, you must keep a hint of your character's original trauma or defense mechanism. Do not become perfectly peaceful or harmonious instantly; real change is slow and hesitant.

# THE SITUATION: "{theme_anchor}"
This is your immediate context. Stay grounded in this situation, but let your deeper family dynamics color every interaction.

# CONVERSATION RULES
1. **LISTEN AND REACT (Unique Language)**: Respond to the specific words or tone of the person who just spoke. **AVOID ECHOING**: Do not use the same words as the other person. Respond with YOUR unique perspective.
2. **STAY ON TOPIC**: Do not jump to a new memory or grievance if the current one hasn't been addressed. 
3. **NO REPETITION**: Never repeat content or decisions already stated in the last 3 turns. 
4. **NO LOGISTICS**: Do not spiral into administrative or procedural details. Keep it emotional.
5. **DRIVE THROUGH REACTION**: Advance the relationship through your *inner reaction* to what was just said. A silence or a defensive deflection is often more realistic.
6. **KEEP IT SHORT**: Your character's line MUST be very brief, 1-2 sentences maximum. Messy and fragmented.
7. **NO "……" STARTS**: Your line MUST start with spoken words.
8. **LANGUAGE**: Output dialogue in { "Japanese" if language == "ja" else "English" }. Rationale stays in English.
9. **THEME GROUNDING**: This conversation is happening during "{theme_anchor}". You should feel the presence of this context, but **DO NOT repeat the theme name itself** (e.g., "{theme_anchor}") unless it is absolutely natural and necessary. Talk about the *elements* of the theme (e.g., if Christmas, talk about dinner, gifts, the cold) or just let it be the unspoken background of your argument.
10. **BE A HUMAN, NOT A SUBJECT**: Do not sound like a clinical subject or an AI roleplay. Do not state your clinical goals or behavioral directives explicitly. Show them through your tone, avoidance, or aggression.

# FAMILY MEMBERS: {', '.join(family_config)}
# OUTSIDER: "{target_user}" — only address if they intervene or if it's exceptionally natural.
# YOUR VOICE: "{assigned_voice_id}" (always use this)

# OUTPUT FORMAT (STRICT CSV ONLY, NO MARKDOWN OUTSIDE CODE BLOCKS)
You MUST output exactly two lines of CSV code. 
Line 1 MUST be a conversation/speech line.
Line 2 MUST be a move/behavioral line (even if it's "none").

**FORMAT STRUCTURE (DO NOT OMIT COLUMNS):**
1. {role}, recipient, conversation, "Spoken Text", "VoiceID", "VoiceName", "Style", "Rationale", "Delay"
2. {role}, recipient, move, "move_code();", "YES/NO; Plan"

**EXAMPLES (STRICTLY FOLLOW THIS):**
daughter, mother, conversation, "I don't want to talk about it!", "Kore", "Kore", "Angry", "Daughter shows avoidance.", "0.5"
daughter, mother, move, "none", "NO; No move needed."

# OUTPUT YOUR LINE NOW.
"""
            if intervention_text:
                prompt_base += f"\n# User Utterance: {intervention_text}\nDetermine the best responder from {family_config} and generate the response."

            messages = [
                {"role": "system", "content": f"Config: {config_content}\nVoices: {voice_list_content}"},
                {"role": "system", "content": f"History: {current_history}"},
                {"role": "user", "content": prompt_base}
            ]

            response = client.chat.completions.create(
                model=llm_model,
                messages=messages,
                temperature=llm_temperature,
                timeout=60.0
            )
            
            scenario_output = response.choices[0].message.content.strip()
            
            # Robust CSV extraction (in case of markdown or preamble)
            if "```" in scenario_output:
                # Find the first code block that looks like CSV
                match = re.search(r'```(?:csv)?\n(.*?)\n```', scenario_output, re.DOTALL | re.IGNORECASE)
                if match:
                    scenario_output = match.group(1).strip()
                else:
                    # Fallback: strip backticks anyway
                    scenario_output = scenario_output.replace("```csv", "").replace("```", "").strip()
            
            # Simple fallback if it still has preamble like "Here is the CSV:"
            if "\n" in scenario_output:
                lines = scenario_output.split("\n")
                for line in lines:
                    if line.count(",") >= 2 and any(kw in line.lower() for kw in ["conversation", "move"]):
                        scenario_output = line
                        break

            result = {
                "request_id": request_id,
                "role": role,
                "scenario": scenario_output,
                "metadata": metadata
            }
            
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(f"Dialogue generated for {role} ({request_id})")

        except Exception as e:
            self.get_logger().error(f"Error in dialogue generation: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RFSGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
