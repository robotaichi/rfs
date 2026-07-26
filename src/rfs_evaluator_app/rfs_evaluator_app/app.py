#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FACES-IV Validation Web App
For external family psychology researchers to validate LLM-generated FACES-IV evaluations.

This file only handles the Flask app itself (routing). ROS2-independent logic
such as the FACES-IV item definitions and conversation history/evaluation CSV
parsing has been split out into `faces_data` (the logic layer).
"""

import os
import platform
import subprocess
from datetime import datetime
from flask import Flask, jsonify, render_template, request

from rfs_evaluator_app.faces_data import (
    FACES_ITEMS,
    SUBSCALES,
    get_subscale,
    parse_conversation_history,
    parse_evaluation_csv,
    parse_conversation_line,
)

# ── Paths ────────────────────────────────────────────────────────────────────
from ament_index_python.packages import get_package_share_directory

try:
    PACKAGE_SHARE_DIR = get_package_share_directory('rfs_evaluator_app')
    STATIC_DIR = os.path.join(PACKAGE_SHARE_DIR, 'static')
    TEMPLATE_DIR = os.path.join(PACKAGE_SHARE_DIR, 'templates')
    # Default archive dir logic: try to find it relative to source if running from source,
    # or use a standard path if installed.
    # Creating a robust fallback for archive dir:
    # 1. Check relative to user home (safest for this environment)
    # 2. Check relative to package location (source)

    # Try user home structure first (most likely for this setup)
    POSSIBLE_ARCHIVE_DIR = os.path.join(os.path.expanduser("~"), "rfs", "src", "rfs_database", "archive")
    if os.path.isdir(POSSIBLE_ARCHIVE_DIR):
        ARCHIVE_DIR = POSSIBLE_ARCHIVE_DIR
    else:
        # Fallback to relative to this file (development mode)
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        ARCHIVE_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "..", "rfs_database", "archive"))

except Exception as e:
    # Fallback for direct python execution without ROS2 environment
    print(f"[WARN] Could not resolve ROS2 package share directory: {e}")
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    # Assuming standard source layout: src/rfs_evaluator_app/rfs_evaluator_app/app.py
    # So static is at src/rfs_evaluator_app/static -> ../../static
    PROJECT_ROOT = os.path.abspath(os.path.join(BASE_DIR, "..", ".."))
    STATIC_DIR = os.path.join(PROJECT_ROOT, "static")
    TEMPLATE_DIR = os.path.join(PROJECT_ROOT, "templates")
    ARCHIVE_DIR = os.path.join(PROJECT_ROOT, "..", "rfs_database", "archive")

app = Flask(__name__, static_folder=STATIC_DIR, template_folder=TEMPLATE_DIR)


# ── Routes ────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/archives")
def list_archives():
    """List available archive folders."""
    if not os.path.isdir(ARCHIVE_DIR):
        return jsonify({"archives": []})

    archives = []
    for name in sorted(os.listdir(ARCHIVE_DIR), reverse=True):
        path = os.path.join(ARCHIVE_DIR, name)
        if os.path.isdir(path):
            # Check required files exist
            has_conv = os.path.isfile(os.path.join(path, "conversation_history.txt"))
            has_eval = os.path.isfile(os.path.join(path, "evaluation_history.csv"))
            if has_conv and has_eval:
                # Parse timestamp for display
                try:
                    dt = datetime.strptime(name, "%Y%m%d_%H%M%S")
                    display = dt.strftime("%Y-%m-%d %H:%M:%S")
                except ValueError:
                    display = name
                archives.append({"name": name, "display": display})
    return jsonify({"archives": archives})


@app.route("/api/archive/<name>")
def get_archive(name: str):
    """Load and return parsed archive data."""
    archive_path = os.path.join(ARCHIVE_DIR, name)
    if not os.path.isdir(archive_path):
        return jsonify({"error": "Archive not found"}), 404

    conv_path = os.path.join(archive_path, "conversation_history.txt")
    eval_path = os.path.join(archive_path, "evaluation_history.csv")

    # Parse conversation history
    with open(conv_path, "r", encoding="utf-8") as f:
        conv_text = f.read()
    sessions_conv = parse_conversation_history(conv_text)

    # Parse into structured lines
    sessions_structured = {}
    for sid, lines in sessions_conv.items():
        sessions_structured[sid] = [parse_conversation_line(l) for l in lines]

    # Parse evaluation scores
    sessions_eval = parse_evaluation_csv(eval_path)

    # Build session list (ordered)
    all_sessions = sorted(set(list(sessions_conv.keys()) + list(sessions_eval.keys())),
                          key=lambda s: int(s[1:]))

    # Build FACES items list
    items = [{"num": k, "text": v, "subscale": get_subscale(k)} for k, v in FACES_ITEMS.items()]

    return jsonify({
        "archive_name": name,
        "sessions": all_sessions,
        "conversations": sessions_structured,
        "evaluations": sessions_eval,
        "items": items,
        "subscales": SUBSCALES,
    })


@app.route("/api/save_csv", methods=["POST"])
def save_csv():
    """Save evaluator's scores as CSV."""
    import csv

    data = request.json
    archive_name = data.get("archive_name")
    evaluator_name = data.get("evaluator_name", "anonymous")
    results = data.get("results", {})  # {session_id: {item_num: score}}

    if not archive_name:
        return jsonify({"error": "Missing archive_name"}), 400

    archive_path = os.path.join(ARCHIVE_DIR, archive_name)
    if not os.path.isdir(archive_path):
        return jsonify({"error": "Archive not found"}), 404

    # Load robot scores
    eval_path = os.path.join(archive_path, "evaluation_history.csv")
    sessions_eval = parse_evaluation_csv(eval_path)

    saved_files = []

    for session_id, evaluator_scores in results.items():
        robot_data = sessions_eval.get(session_id, {"members": {}, "mean": {}})
        members = robot_data["members"]
        mean_scores = robot_data["mean"]
        member_names = sorted(members.keys())

        # Build CSV
        filename = f"human_evaluation_{session_id}_{evaluator_name}.csv"
        filepath = os.path.join(archive_path, filename)

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            header = ["Item", "Item_Text", "Subscale"] + member_names + ["robot_mean", "evaluator"]
            writer.writerow(header)

            for item_num in range(1, 63):
                item_key = str(item_num)
                row = [
                    item_num,
                    FACES_ITEMS.get(item_num, ""),
                    get_subscale(item_num),
                ]
                for member in member_names:
                    score = members.get(member, {}).get(item_key, "")
                    row.append(score)
                row.append(round(float(mean_scores.get(item_key, 0)), 2))
                row.append(evaluator_scores.get(item_key, ""))
                writer.writerow(row)

        saved_files.append(filepath)

    return jsonify({"saved_files": saved_files})


@app.route("/api/open_file", methods=["POST"])
def open_file():
    """Open file in OS file explorer with the file selected."""
    data = request.json
    filepath = data.get("filepath", "")

    if not os.path.isfile(filepath):
        return jsonify({"error": "File not found"}), 404

    system = platform.system()
    try:
        if system == "Windows":
            subprocess.Popen(["explorer", "/select,", filepath.replace("/", "\\")])
        elif system == "Darwin":
            subprocess.Popen(["open", "-R", filepath])
        else:  # Linux
            parent_dir = os.path.dirname(filepath)
            subprocess.Popen(["xdg-open", parent_dir])
        return jsonify({"status": "ok", "system": system})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"[INFO] Archive directory: {ARCHIVE_DIR}")
    print(f"[INFO] Starting FACES-IV Validation App on http://localhost:5001")
    app.run(host="0.0.0.0", port=5001, debug=True)

if __name__ == "__main__":
    main()
