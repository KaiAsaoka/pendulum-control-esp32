# TelemetryConfigV1_0.py

import json
import os
import sys

CONFIG_PATH = "config.json"

if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)
else:
    sys.exit("Failed to load config")

ESP_IP = config["ESP_IP"]  # Change to your ESP32 IP
UDP_IP = config["UDP_IP"]         # Listen on all interfaces
UDP_PORT = config["UDP_PORT"]

# ----------------- DATA BUFFER SETTINGS -----------------
MAX_POINTS = config["MAX_POINTS"]  # maximum points stored per variable

# ----------------- TIMEBASE SETTINGS (OSCILLOSCOPE-LIKE) -----------------
TIME_PER_DIV_DEFAULT = config["TIME_PER_DIV"]  # ms per division
NUM_DIVS_DEFAULT = config["NUM_DIVS"]       # number of horizontal divisions
