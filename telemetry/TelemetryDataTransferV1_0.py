import socket
import struct
import threading
from collections import deque
from time import sleep
import serial
import serial.tools.list_ports

from TelemetryConfigV1_0 import MAX_POINTS

# ----------------- GLOBALS -----------------
sock = None
ser = None
variable_names = []
data_buffers = {}
selected_vars = []
use_serial = False
sending_pid = False

pause_receive = threading.Event()

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
    ser.write(b"METADATA")
    while True:
        data = ser.readline()
        print(data)
        if len(data) < 3:
            sleep(0.05)
            continue

        if data[0] == 0xCD and data[1] == 0xAB:
            num_vars = data[2]
            offset = 3
            names = []
            for _ in range(num_vars):
                # print(offset)
                name_len = data[offset]
                offset += 1
                name = data[offset:offset+name_len].decode('ascii')
                # print(name)
                offset += name_len
                names.append(name)
            print("Metadata received (Serial)! Variable names:", names)
            return names, None
        
def receive_pid():
    ser.reset_input_buffer()
    ser.write(b"SENDPID")
    print("Request PID")

    expected_bytes = 3 + 20 * 4  # header + len + 20 ints
    while True:
        # buffer = b""

        # keep reading until we get the full packet
        buffer = ser.readline()
        # while len(buffer) < expected_bytes:
        #     chunk = ser.read(expected_bytes - len(buffer))
        #     if chunk:
        #         buffer += chunk
        #     else:
        #         sleep(0.01)

        print(buffer)
        # validate header
        if buffer[0] != 0xcd or buffer[1] != 0xac:
            print("Invalid header:", buffer[:4])
            sleep(0.05)
            continue
        else:
            offset = 3
            pid_vals = struct.unpack("<20i", buffer[offset:offset + 80])
            return list(pid_vals)

# ----------------- RECEIVE TELEMETRY -----------------
def receive_telemetry(num_vars, variable_names, data_buffers):
    global sending_pid
    snapshot_struct = "<" + "f"*num_vars
    snapshot_size = 4*num_vars + 8

    buffer = b""
    while True:
        if pause_receive.is_set():
            continue
        
        while(sending_pid):
            sleep(0.05)

        new_data = ser.readline()
        #print(new_data)
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
            packet_size = 6 + num_snapshots * (4*num_vars + 8) + 3

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

# ----------------- SEND PID -----------------
def send_pid(pid_vals):
    global sending_pid
    stop_telemetry()
    sleep(0.1)

    sending_pid = True

    axes = ["Set Angle X", "Set Angle Y", "Set PWM X", "Set PWM Y"]
    params = ["P", "I", "D", "LPF", "Windup"]
    ordered_vals = [pid_vals[f"{axis}_{param}"] for axis in axes for param in params]

    ser.write(b"PIDRECV")
    print("Sent PIDRECV")

    while True:
        resp = ser.readline()
        print("ESP32 Response:", resp)
        if b"PID received!" not in resp:
            print("Unexpected response:", resp)
            continue
        
        payload = struct.pack("<20f", *ordered_vals)
        ser.write(payload)
        print(payload)
        print("PID values sent!")

        sending_pid = False
        return

# ----------------- CONTROL COMMANDS -----------------
def start_telemetry(variable_names, esp_addr=None):
    for name in variable_names:
        data_buffers[name] = deque(maxlen=MAX_POINTS)

    ser.write(b"START")
    print("START command sent (Serial).")

    pause_receive.clear()
    thread = threading.Thread(target=receive_telemetry, args=(len(variable_names), variable_names, data_buffers), daemon=True)
    thread.start()
    if thread.is_alive():
        print("Receiver thread started.")
    # print(data_buffers)
    return data_buffers

def send_pulse():
    ser.write(b"PULSE")
    # print("PULSE command sent (Serial).")

def stop_telemetry():
    ser.write(b"STOP")
    pause_receive.set()
    sleep(0.05)

def simple_start():
    ser.write(b"START")
    pause_receive.clear()
    sleep(0.05)

# ----------------- MAIN ENTRY -----------------
if __name__ == "__main__":
    setup_serial()
    variable_names = receive_metadata()

    # ----- Start Telemetry -----
    start_telemetry(variable_names)

    # Keep alive pulses