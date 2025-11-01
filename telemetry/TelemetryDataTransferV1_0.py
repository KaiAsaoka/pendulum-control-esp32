import socket
import struct
import threading
from collections import deque
from time import sleep
import serial
import serial.tools.list_ports

from TelemetryConfigV1_0 import UDP_IP, UDP_PORT, ESP_IP, MAX_POINTS

# ----------------- GLOBALS -----------------
sock = None
ser = None
variable_names = []
data_buffers = {}
selected_vars = []
use_serial = False

# ----------------- UDP SETUP -----------------
def setup_udp():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)
    print(f"UDP listening on {UDP_IP}:{UDP_PORT}")

# ----------------- SERIAL SETUP -----------------
def setup_serial(port=None, baudrate=115200):
    global ser, use_serial
    use_serial = True

    if port is None:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            raise RuntimeError("No serial ports detected!")
        print("Available serial ports:")
        for p in ports:
            print(f" - {p.device}")
        port = ports[0].device
        print(f"Using first port: {port}")

    ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
    print(f"Serial connected on {port} at {baudrate} baud")

# ----------------- RECEIVE METADATA -----------------
def receive_metadata():
    if use_serial:
        return receive_metadata_serial()
    else:
        return receive_metadata_udp()

def receive_metadata_udp():
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
            print("Metadata received (UDP)! Variable names:", names)
            return names, addr

def receive_metadata_serial():
    ser.write(b"METADATA\n")
    while True:
        data = ser.read_until(b'\xFF\xFF')
        if len(data) < 3:
            sleep(0.05)
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
            print("Metadata received (Serial)! Variable names:", names)
            return names, None

# ----------------- RECEIVE TELEMETRY -----------------
def receive_telemetry(num_vars, variable_names, data_buffers):
    if use_serial:
        receive_telemetry_serial(num_vars, variable_names, data_buffers)
    else:
        receive_telemetry_udp(num_vars, variable_names, data_buffers)

def receive_telemetry_udp(num_vars, variable_names, data_buffers):
    snapshot_struct = "<" + "f"*num_vars
    snapshot_size = 4*num_vars + 8

    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except (socket.timeout):
            continue
        except (ConnectionResetError): 
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
                data_buffers[name].append((timestamp_us/1000.0, val))

def receive_telemetry_serial(num_vars, variable_names, data_buffers):
    snapshot_struct = "<" + "f"*num_vars
    snapshot_size = 4*num_vars + 8

    buffer = b""
    while True:
        new_data = ser.read(4096)
        if not new_data:
            continue
        buffer += new_data

        while len(buffer) >= 6:
            sync = struct.unpack_from("<H", buffer, 0)[0]
            if sync != 0xAA55:
                buffer = buffer[1:]
                continue

            if len(buffer) < 6:
                break

            _, seq, num_snapshots, num_vars_in_packet = struct.unpack_from("<HHBB", buffer, 0)
            packet_size = 6 + num_snapshots * (4*num_vars + 8) + 2

            if len(buffer) < packet_size:
                break

            packet = buffer[:packet_size]
            buffer = buffer[packet_size:]

            offset = 6
            for _ in range(num_snapshots):
                if offset + snapshot_size > len(packet):
                    break
                vars_values = list(struct.unpack_from(snapshot_struct, packet, offset))
                offset += 4*num_vars
                timestamp_us = struct.unpack_from("<Q", packet, offset)[0]
                offset += 8
                for i, val in enumerate(vars_values):
                    name = variable_names[i]
                    data_buffers[name].append((timestamp_us/1000.0, val))

# ----------------- CONTROL COMMANDS -----------------
def start_telemetry(variable_names, esp_addr=None):
    for name in variable_names:
        data_buffers[name] = deque(maxlen=MAX_POINTS)

    if use_serial:
        ser.write(b"START\n")
        print("START command sent (Serial).")
    else:
        sock.sendto(b"START", esp_addr)
        print(f"START command sent to {esp_addr}, ESP should begin transmitting...")
        send_pulse(esp_addr)

    thread = threading.Thread(target=receive_telemetry, args=(len(variable_names), variable_names, data_buffers), daemon=True)
    thread.start()
    if thread.is_alive():
        print("Receiver thread started.")
    return data_buffers

def send_pulse(esp_addr=None):
    if use_serial:
        ser.write(b"PULSE\n")
        print("PULSE command sent (Serial).")
    else:
        sock.sendto(b"PULSE", esp_addr)
        print(f"PULSE command sent to {esp_addr} (UDP).")

# ----------------- MAIN ENTRY -----------------
if __name__ == "__main__":
    # ----- Choose Communication Type -----
    mode = input("Enter mode (udp/serial): ").strip().lower()
    if mode == "serial":
        setup_serial()  # auto-select first available
    else:
        setup_udp()

    # ----- Get Metadata -----
    variable_names, esp_addr = receive_metadata()

    # ----- Start Telemetry -----
    start_telemetry(variable_names, esp_addr)

    # Keep alive pulses
    while True:
        sleep(1)
        if not use_serial:
            send_pulse(esp_addr)
        else:
            send_pulse()