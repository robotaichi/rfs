#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Support logic layer for the family member node.

Handles role-name normalization, reading/writing the conversation history file,
responder voting (LLM call), and voice assignment. Does not depend on ROS2
communication. Called from the rfs_family_member node.

The conversation state changes themselves (when to publish what, to whom) are
tied closely to locks/timers, so splitting them out risks breaking the timing;
they are therefore not included here (per the CLAUDE.md policy on what stays
in the Node).
"""

import fcntl
import json
import os


# Lookup table that turns Japanese/English ways of addressing a family member into the standard role name
_ROLE_ALIAS_LOOKUP = {
    "父": "father", "お父さん": "father", "お父ちゃん": "father", "パパ": "father", "親父": "father",
    "母": "mother", "お母さん": "mother", "お母ちゃん": "mother", "ママ": "mother", "お袋": "mother",
    "娘": "daughter", "夏菜": "daughter", "お姉ちゃん": "daughter", "お姉さん": "daughter", "姉": "daughter",
    "息子": "son", "和也": "son", "お兄ちゃん": "son", "お兄さん": "son", "兄": "son", "弟": "son", "妹": "son",
    "祖父": "grandpa", "おじいちゃん": "grandpa", "おじいさん": "grandpa",
    "祖母": "grandma", "おばあちゃん": "grandma", "おばあさん": "grandma",
}


def normalize_role_name(name: str) -> str:
    """Normalize a role name (possibly a Japanese form of address, etc.) to the standard role name (lowercase English)."""
    if not name:
        return ""
    n = name.lower().strip()
    return _ROLE_ALIAS_LOOKUP.get(n, n)


def load_full_history(history_file: str) -> str:
    """Load the conversation history file. Excludes therapist-analysis and system-update lines to preserve the character's "fourth wall"."""
    try:
        with open(history_file, "r", encoding="utf-8") as f:
            lines = f.readlines()
        filtered = [l for l in lines if not (l.startswith("[THERAPIST_") or l.startswith("[SYSTEM_UPDATE"))]
        return "".join(filtered)
    except FileNotFoundError:
        return ""


def get_turn_count(history_file: str) -> int:
    """Count the number of "conversation" lines in the history file (= number of dialogue turns)."""
    count = 0
    if not os.path.exists(history_file):
        return 0
    try:
        with open(history_file, "r", encoding="utf-8") as f:
            for line in f:
                if "conversation" in line:
                    count += 1
    except Exception:
        pass
    return count


def append_history(history_file: str, text: str, turns_per_step: int, is_leader_response: bool, logger=None):
    """Append one line to the conversation history file, using flock so two processes can't write to it at the same time."""
    if text is None:
        return

    turns = get_turn_count(history_file)
    step_id = turns // turns_per_step

    write_text = text
    if not text.startswith("[SYSTEM_UPDATE]") and ("conversation" in text or "move" in text):
        write_text = f"S{step_id}_T{turns+1},{text}"

    prefix = "[LEADER_RESPONSE] " if is_leader_response else ""
    try:
        with open(history_file, "a", encoding="utf-8") as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            f.write(prefix + write_text + "\n")
            f.flush()
            try:
                os.fsync(f.fileno())
            except Exception:
                pass
            fcntl.flock(f, fcntl.LOCK_UN)
    except Exception as e:
        if logger:
            logger.error(f"Failed to write history: {e}")


def cast_responder_vote(role: str, family_config: list, language: str, history_text: str, user_text: str, logger=None) -> str:
    """Ask the Gemini API to decide who should respond to the user's utterance, and return the voted-for role name."""
    try:
        import requests
        import socket
        import urllib3.util.connection as urllib3_cn
        urllib3_cn.allowed_gai_family = lambda: socket.AF_INET

        # Only use last 20 lines for context efficiency
        history_lines = history_text.strip().split('\n')[-20:]
        recent_history = '\n'.join(history_lines)

        lang_hint = "日本語の会話です。" if language == "ja" else ""
        prompt = f"""
{lang_hint}
You are "{role}" in a family simulation with members: {family_config}.
A user (outsider) just said: "{user_text}"

Recent conversation:
{recent_history}

Based on the user's speech and the conversation context, which family member is the MOST appropriate to respond to the user?
Consider:
- Who is most relevant to what the user said?
- Who was most recently involved in the conversation topic?
- Who has the personality/role best suited to respond?

Respond with ONLY the name of ONE family member from this list: {family_config}
Output the name in lowercase, nothing else.
"""
        api_key = os.environ.get('GEMINI_API_KEY')
        if not api_key:
            return family_config[0]

        url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-3.1-flash-lite:generateContent?key={api_key}"
        headers = {"Content-Type": "application/json"}
        payload = {
            "contents": [{"parts": [{"text": prompt}]}],
            "generationConfig": {"temperature": 0.0}
        }
        res = requests.post(url, headers=headers, json=payload, timeout=15.0)
        if res.status_code == 200:
            ans = res.json()["candidates"][0]["content"]["parts"][0]["text"].strip().lower()
        else:
            if logger:
                logger.error(f"[{role}] Vote API error: {res.status_code}")
            return family_config[0]

        for m in family_config:
            if m.lower() in ans:
                return m.lower()
        return family_config[0]
    except Exception as e:
        if logger:
            logger.error(f"[{role}] Error casting vote: {e}")
        return family_config[0]


def assign_voice(role: str, voice_list_file: str, logger=None) -> str:
    """Guess the gender from the role name, then always pick the same voice for that gender using a hash of the role name."""
    try:
        with open(voice_list_file, 'r', encoding='utf-8') as f:
            v_list = json.load(f)

        # Reliable gender detection (works for English/Japanese and common family roles)
        gender = "female"
        male_keywords = ["father", "son", "brother", "grandpa", "grandfather", "uncle", "boy", "man", "male",
                         "父", "父さん", "お父さん", "パパ", "息子", "兄", "弟", "おじいさん", "おじいちゃん", "祖父", "叔父", "伯父", "男"]
        if any(k in role.lower() for k in male_keywords):
            gender = "male"

        # Extra rule: roles containing 'daughter', 'mother', 'girl', etc. are always female
        female_keywords = ["mother", "daughter", "sister", "grandma", "grandmother", "aunt", "girl", "woman", "female",
                           "母", "母さん", "お母さん", "ママ", "娘", "姉", "妹", "おばあさん", "おばあちゃん", "祖母", "叔母", "伯母", "女"]
        if gender == "male" and any(k in role.lower() for k in female_keywords):
            gender = "female"

        candidates = [v for v in v_list if v.get("gender", "").lower() == gender]
        if not candidates:
            candidates = v_list

        # Deterministic voice assignment based on role hash (no API call needed)
        role_hash = sum(ord(c) for c in role)
        selected = candidates[role_hash % len(candidates)]
        if logger:
            logger.info(f"[{role}] Voice assigned: {selected['name']} (gender={gender})")
        return selected['name']
    except Exception as e:
        if logger:
            logger.error(f"Voice assignment error: {e}")
        return "Kore"
