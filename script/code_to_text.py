#!/usr/bin/env python3

"""
A System to turn python code into text files
For Assignment Submission
Author: Michael Franks
"""

import os

# Configuration
STUDENT_NUMBER = "23006083"
OUTPUT_FILE = f"Script{STUDENT_NUMBER}.txt"
# Based on your image, the code lives in src/robot/robot
SOURCE_PATH = os.path.expanduser("~/ros2_ws/src/robot/robot")


def generate_text_submission():
    if not os.path.exists(SOURCE_PATH):
        print(f"Error: Could not find directory {SOURCE_PATH}")
        return

    with open(OUTPUT_FILE, "w", encoding="utf-8") as outfile:
        outfile.write(f"# PLAIN TEXT CODE SUBMISSION - STUDENT {STUDENT_NUMBER}\n")
        outfile.write("# =====================================================\n")

        file_count = 0

        # Walk through all subdirectories (data, modules, utils, etc.)
        for root, _, files in os.walk(SOURCE_PATH):
            for filename in sorted(files):
                # Logic: Only include Python files, ignore __init__ and this script itself
                if filename.endswith(".py") and "__init__" not in filename:
                    if filename == os.path.basename(__file__):
                        continue

                    filepath = os.path.join(root, filename)

                    # Create a clear relative path for the header (e.g., modules/actuator.py)
                    relative_path = os.path.relpath(filepath, SOURCE_PATH)

                    outfile.write(f"\n# === {relative_path} ===\n")
                    outfile.write("-" * (len(relative_path) + 10) + "\n")

                    try:
                        with open(filepath, "r", encoding="utf-8") as infile:
                            outfile.write(infile.read())
                        outfile.write("\n\n")
                        print(f"Aggregated: {relative_path}")
                        file_count += 1
                    except Exception as e:
                        print(f"Error reading {filename}: {e}")

    print(f"\nSuccess! {file_count} files consolidated into {OUTPUT_FILE}")


if __name__ == "__main__":
    generate_text_submission()
