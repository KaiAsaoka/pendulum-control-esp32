import socket
import struct
import threading
from collections import deque
from time import sleep

from TelemetryConfigV1_0 import UDP_IP, UDP_PORT, ESP_IP, MAX_POINTS

# ----------------- UDP SOCKET -----------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

variable_names = []
data_buffers = {}
selected_vars = []

# ----------------- UDP RECEIVERS -----------------
def receive_metadata():
    while True:
        try:
            sock.sendto(b"METADATA", (ESP_IP, UDP_PORT))
        except Exception:
            pass

        try:
            data, addr = sock.recvfrom(512)
        except (socket.timeout, ConnectionResetError):
            sleep(0.05)
            continue

        if len(data) < 3:
            continue

        if data[0] == 0xCD and data[1] == 0xAB:
            num_vars = data[2]
            offset = 3
            names = []
            for _ in range(num_vars):
                name_len = data[offset]
                offset += 1
                name = data[offset:offset+name_len].decode('ascii')
                offset += name_len
                names.append(name)
            print("Metadata received! Variable names:", names)
            return names, addr

def receive_telemetry(num_vars, variable_names, data_buffers):
    snapshot_struct = "<" + "f"*num_vars
    snapshot_size = 4*num_vars + 8

    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except (socket.timeout, ConnectionResetError):
            continue

        if len(data) < 6:
            continue

        sync, seq, num_snapshots, num_vars_in_packet = struct.unpack_from("<HHBB", data, 0)
        if sync != 0xAA55:
            continue
        offset = 6

        for _ in range(num_snapshots):
            if offset + snapshot_size > len(data):
                break

            vars_values = list(struct.unpack_from(snapshot_struct, data, offset))
            offset += 4*num_vars

            timestamp_us = struct.unpack_from("<Q", data, offset)[0]
            offset += 8

            for i, val in enumerate(vars_values):
                name = variable_names[i]
                print(name)
                print(val)
                ## REMOVE THIS LATER ###################
                if name == "carriageXPosition":
                    print("X Position: ", val)
                elif name == "carriageYPosition":
                    print("Y Position: ", val)
                ########################################
                data_buffers[name].append((timestamp_us/1000.0, val))  # ms

def start_telemetry(variable_names, esp_addr):
    for name in variable_names:
        data_buffers[name] = deque(maxlen=MAX_POINTS)

    sock.sendto(b"START", esp_addr)
    print(f"Start command sent to {esp_addr}, ESP should begin transmitting...")

    thread = threading.Thread(target=receive_telemetry, args=(len(variable_names), variable_names, data_buffers), daemon=True)
    thread.start()
    return data_buffers

def send_pulse(esp_addr):
    sock.sendto(b"PULSE", esp_addr)
    print(f"PULSE command sent to {esp_addr}, Lets ESP Know We're Still Listening...")
