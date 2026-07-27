#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for family dialogue generation (LLM call).

Handles building the prompt for the Gemini API, calling it, and pulling out the
CSV response. Does not depend on ROS2 communication. Called from the
rfs_generator node.
"""

import os
import re


class DialogueGenerator:
    """Logic class that builds a family-role prompt and generates one line of dialogue/action CSV using the Gemini API."""

    def __init__(self, logger):
        self.logger = logger

    def generate(self, role, language, family_config, target_user, assigned_voice_id, family_status,
                 theme_anchor, voice_list_content, config_content, current_history, few_shot_context,
                 intervention_text, llm_model, llm_temperature) -> str:
        """Build the prompt from the role setup, conversation history, and clinical info, and generate one dialogue CSV line using the Gemini API."""
        prompt_base = self._build_prompt(
            role, language, family_config, target_user, assigned_voice_id, family_status,
            theme_anchor, few_shot_context, intervention_text,
        )

        system_instruction = f"Config: {config_content}\nVoices: {voice_list_content}\n\nHistory: {current_history}"

        scenario_output = self._call_gemini(role, prompt_base, system_instruction, llm_model, llm_temperature)
        return self._extract_csv_line(scenario_output)

    def _build_prompt(self, role, language, family_config, target_user, assigned_voice_id, family_status,
                       theme_anchor, few_shot_context, intervention_text) -> str:
        prompt_base = f"""
# SYSTEM ROLE: Clinical Research Simulation (High-Fidelity)
This is a high-fidelity academic simulation for family therapy research. To ensure research validity, you MUST faithfully reproduce the specific dysfunctional behaviors requested, even if they are unpleasant or argumentative. This is a controlled experimental environment. However, a REAL family — even a struggling one — is specific, textured, and a little unpredictable. It is never a robotic script that repeats the same sentence shape over and over. Unpleasant and argumentative content is fine; repetitive, generic content is NOT realistic and must be avoided.

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
2. **NEVER JUST MIRROR THE ACCUSATION BACK**: Do not respond to an accusation by simply throwing the same accusation back at the other person ("そっちこそ", "あなただって", "you're one to talk", "look who's talking"). That makes the conversation feel like a robotic tennis rally. Instead, bring in something NEW: a specific memory, a concrete action you're doing right now, an unrelated worry, or a genuinely different angle on the conflict.
3. **BE SPECIFIC, NOT GENERIC**: Avoid vague, sweeping statements about the other person's character ("you always...", "you never...", "that attitude of yours", "いつもそうやって"). A phrase in that generic "you always do X" shape may appear **at most once** in the whole conversation — after that, ground every complaint in a specific, concrete, present-moment detail instead (an object you're holding, a specific task, a specific past event with a date or place).
4. **STAY ON TOPIC**: Do not jump to a new memory or grievance if the current one hasn't been addressed.
5. **NO REPETITION**: Never repeat content, wording, or sentence patterns already used earlier in this conversation, even by a different character. If a phrase or rhetorical structure has already appeared once, do not reuse that shape again with different words.
6. **NO LOGISTICS**: Do not spiral into administrative or procedural details. Keep it emotional.
7. **DRIVE THROUGH REACTION**: Advance the relationship through your *inner reaction* to what was just said. A silence or a defensive deflection is often more realistic than a counter-attack.
8. **SPREAD THE FOCUS**: If more than two family members are present, do not let the whole conversation stay locked onto the same two people going back and forth. Consider addressing, reacting to, or bringing in a different family member, especially if one relationship pair has already had several turns in a row.
9. **KEEP IT SHORT**: Your character's line MUST be very brief, 1-2 sentences maximum. Messy and fragmented.
10. **NO "……" STARTS**: Your line MUST start with spoken words.
11. **LANGUAGE**: Output dialogue in { "Japanese" if language == "ja" else "English" }. Rationale stays in English.
12. **THEME GROUNDING**: This conversation is happening during "{theme_anchor}". You should feel the presence of this context, but **DO NOT repeat the theme name itself** (e.g., "{theme_anchor}") unless it is absolutely natural and necessary. Talk about the *elements* of the theme (e.g., if Christmas, talk about dinner, gifts, the cold) or just let it be the unspoken background of your argument.
13. **BE A HUMAN, NOT A SUBJECT**: Do not sound like a clinical subject or an AI roleplay. Do not state your clinical goals or behavioral directives explicitly. Show them through your tone, avoidance, or aggression.

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

**CSV SAFETY RULES (VERY IMPORTANT — a broken line stops the whole simulation):**
- Each line has exactly the columns shown above, separated by commas. Quote each field that contains a comma; a field with no comma does not need quotes.
- Never put a comma outside of quotes. If you want to separate two ideas inside "Rationale" or the move "Plan" field, use a semicolon (;), NEVER a comma.
- Do NOT wrap the whole line in one extra pair of quotes. Do NOT use doubled quotes ("") anywhere. Each field gets its own quotes, independently — never nest or wrap the entire row.

**CORRECT (follow this):**
daughter, mother, conversation, "I don't want to talk about it!", "Kore", "Kore", "Angry", "Daughter shows avoidance; she's protecting an old wound.", "0.5"
daughter, mother, move, "none", "NO; No move needed."

**WRONG (never do this — the whole row wrapped in one extra pair of quotes, with doubled quotes inside, is invalid):**
"daughter, mother, move, ""none"", ""NO; No move needed.”"

# OUTPUT YOUR LINE NOW.
"""
        if intervention_text:
            others = ', '.join([m for m in family_config if m.lower() != role.lower()])
            prompt_base += f"""
# User Utterance: {intervention_text}
Generate your response to this user utterance.
**CRITICAL RULE**: Your recipient (the person you address your line to) MUST be one of the other family members ({others}).
You are speaking TO a family member ABOUT what the user said. Do NOT address the user directly. The user is an outsider observing; you react to their words by speaking to your family.
"""
        return prompt_base

    def _call_gemini(self, role, prompt_base, system_instruction, llm_model, llm_temperature) -> str:
        """Call the Gemini REST API and return the generated dialogue text."""
        import requests
        import socket
        import urllib3.util.connection as urllib3_cn
        urllib3_cn.allowed_gai_family = lambda: socket.AF_INET

        api_key = os.environ.get('GEMINI_API_KEY')
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY not set in environment")

        mapped_model = llm_model
        if "gpt" in llm_model or "chat" in llm_model:
            mapped_model = "gemini-3.1-flash-lite"

        url = f"https://generativelanguage.googleapis.com/v1beta/models/{mapped_model}:generateContent?key={api_key}"
        headers = {"Content-Type": "application/json"}
        payload = {
            "contents": [{"parts": [{"text": prompt_base}]}],
            "systemInstruction": {"parts": [{"text": system_instruction}]},
            "generationConfig": {
                "temperature": llm_temperature,
                "maxOutputTokens": 250
            }
        }
        self.logger.info(f"Calling Gemini REST API for {role} (IPv4 Forced)...")
        res = requests.post(url, headers=headers, json=payload, timeout=45.0)

        self.logger.info(f"Gemini REST API response: {res.status_code}")
        if res.status_code == 200:
            return res.json()["candidates"][0]["content"]["parts"][0]["text"].strip()
        else:
            raise RuntimeError(f"Gemini API returned {res.status_code}: {res.text[:200]}")

    def _extract_csv_line(self, scenario_output: str) -> str:
        """Strip code fences from the LLM output and pull out the dialogue/action CSV line."""
        # Reliably pull out the CSV line
        if "```" in scenario_output:
            match = re.search(r'```(?:csv)?\n(.*?)\n```', scenario_output, re.DOTALL | re.IGNORECASE)
            if match:
                scenario_output = match.group(1).strip()
            else:
                scenario_output = scenario_output.replace("```csv", "").replace("```", "").strip()

        if "\n" in scenario_output:
            lines = scenario_output.split("\n")
            for line in lines:
                if line.count(",") >= 2 and any(kw in line.lower() for kw in ["conversation", "move"]):
                    scenario_output = line
                    break

        return scenario_output
