#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logic layer for FACES-IV data definitions and parsing.

Holds the 62-item definitions and subscale grouping, and parses the
conversation history/evaluation CSV. Does not depend on Flask (this is a
ROS2-independent web app already). Called from app.py in rfs_evaluator_app.
"""

import csv
import json
import re

# ── FACES-IV 62 Items ────────────────────────────────────────────────────────
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
    50: "Family members try to understand each other's feelings.",
    51: "When angry, family members seldom say negative things about each other.",
    52: "Family members express their true feelings to each other.",
    53: "The degree of closeness between family members.",
    54: "Your family's ability to cope with stress.",
    55: "Your family's ability to be flexible.",
    56: "Your family's ability to share positive experiences.",
    57: "The quality of communication between family members.",
    58: "Your family's ability to resolve conflicts.",
    59: "The amount of time you spend together as a family.",
    60: "The way problems are discussed.",
    61: "The fairness of criticism in your family.",
    62: "Family members concern for each other.",
}

# FACES-IV subscale groupings
SUBSCALES = {
    "Balanced Cohesion": [1, 7, 13, 19, 25, 31, 37],
    "Balanced Flexibility": [2, 8, 14, 20, 26, 32, 38],
    "Disengaged": [3, 9, 15, 21, 27, 33, 39],
    "Enmeshed": [4, 10, 16, 22, 28, 34, 40],
    "Rigid": [5, 11, 17, 23, 29, 35, 41],
    "Chaotic": [6, 12, 18, 24, 30, 36, 42],
    "Communication": list(range(43, 53)),
    "Satisfaction": list(range(53, 63)),
}


def get_subscale(item_num: int) -> str:
    """Return the subscale name for a given item number."""
    for name, items in SUBSCALES.items():
        if item_num in items:
            return name
    return "Unknown"


def parse_conversation_history(text: str) -> dict:
    """Parse conversation_history.txt → {session_id: [lines]}."""
    sessions = {}
    current_session = None
    s_pattern = re.compile(r"^(S\d+)_T\d+")

    for line in text.strip().split("\n"):
        line = line.strip()
        if not line:
            continue
        # Skip therapist analysis blocks
        if line.startswith("[THERAPIST_"):
            continue
        # Check if it's metadata (target scores etc.)
        if line.startswith("Determined ") or line.startswith("Targeted ") or \
           line.startswith("- ") or line.startswith("(Strategic"):
            continue

        m = s_pattern.match(line)
        if m:
            sid = m.group(1)
            current_session = sid
            if sid not in sessions:
                sessions[sid] = []
            sessions[sid].append(line)

    return sessions


def parse_evaluation_csv(filepath: str) -> dict:
    """Parse evaluation_history.csv → {session_id: {member: {item: score}}}."""
    sessions = {}
    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            step_id = row.get("StepID", "").strip()
            if not step_id:
                continue
            raw_json = row.get("Member_Raw_Scores_JSON", "{}")
            mean_json = row.get("Mean_Ratings_JSON", "{}")
            try:
                member_scores = json.loads(raw_json)
                mean_scores = json.loads(mean_json)
            except json.JSONDecodeError:
                member_scores = {}
                mean_scores = {}
            sessions[step_id] = {
                "members": member_scores,
                "mean": mean_scores,
            }
    return sessions


def parse_conversation_line(line: str) -> dict:
    """Parse a single conversation line into structured data."""
    # Format: S0_T1,daughter,father,conversation,<text>,Leda,Leda,Normal,"<rationale>",0.5
    parts = line.split(",", 9)
    if len(parts) < 5:
        return {"raw": line}
    return {
        "step": parts[0],
        "speaker": parts[1],
        "target": parts[2],
        "type": parts[3],
        "text": parts[4],
    }
