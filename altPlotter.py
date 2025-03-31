#!/usr/bin/env python3
"""
Example: "Our Current Code" with multi-thread + queue approach,
updated for 6 input fields in the format:

  E1: <val>, E2: <val>, G1: <val>, G2: <val>, xVel: <val>, yVel: <val>

We produce two 2D visuals:
  1) GantryVisualizer:  (G1/G2 vs time) and (xVel/yVel vs time)
  2) PendulumVisualizer: (E1/E2 vs time) and a tilt plot showing (E2,E1)

Author: [Your Name]
Date: [Date]
"""

import time
import math
import threading
import queue
import serial

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# -------------
# User Config
# -------------
SERIAL_PORT = 'COM3'        # Adjust if needed
BAUD_RATE   = 115200
UPDATE_INTERVAL_MS = 50     # ~20 FPS
MAX_POINTS  = 1000          # store up to 1000 samples

# -----------------------
# Data Store
# -----------------------
class DataStore:
    """
    Holds the time series for 6 fields we care about:
        E1, E2, G1, G2, xVel, yVel
    plus time stamps.
    """
    def __init__(self, maxlen=1000):
        self.maxlen = maxlen
        self.start_time = time.time()

        self.times = []
        self.E1    = []
        self.E2    = []
        self.G1    = []
        self.G2    = []
        self.xVel  = []
        self.yVel  = []

    def append(self, e1, e2, g1, g2, xv, yv):
        now = time.time() - self.start_time

        def store(lst, val):
            lst.append(val)
            if len(lst) > self.maxlen:
                lst.pop(0)

        store(self.times, now)
        store(self.E1,    e1)
        store(self.E2,    e2)
        store(self.G1,    g1)
        store(self.G2,    g2)
        store(self.xVel,  xv)
        store(self.yVel,  yv)

# -----------------------
# Parsing
# -----------------------
def parse_line(line):
    """
    Expects a line in the form:
      E1: <val>, E2: <val>, G1: <val>, G2: <val>, xVel: <val>, yVel: <val>

    Returns (E1, E2, G1, G2, xVel, yVel) as floats, or None on parse failure.
    """
    line = line.strip()
    if not line:
        return None

    # Split by commas
    parts = line.split(',')
    if len(parts) < 6:
        return None  # not enough fields

    try:
        # For each part, split on ":", take the second half, strip() it, convert to float
        # part[0] => "E1: 123.45"
        e1_str = parts[0].split(':')[1].strip()
        e2_str = parts[1].split(':')[1].strip()
        g1_str = parts[2].split(':')[1].strip()
        g2_str = parts[3].split(':')[1].strip()
        xv_str = parts[4].split(':')[1].strip()
        yv_str = parts[5].split(':')[1].strip()

        e1_val = float(e1_str)
        e2_val = float(e2_str)
        g1_val = float(g1_str)
        g2_val = float(g2_str)
        xv_val = float(xv_str)
        yv_val = float(yv_str)

        return (e1_val, e2_val, g1_val, g2_val, xv_val, yv_val)
    except:
        return None

# -----------------------
# Threads
# -----------------------
def serial_reader_thread(ser, line_queue, stop_event):
    """
    Continuously reads raw lines from serial,
    places them into line_queue for processing.
    """
    buffer = ""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting).decode(errors='replace')
                buffer += chunk
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line_queue.put(line.strip())
        except Exception as e:
            print(f"[ReaderThread] Error: {e}")
        time.sleep(0.001)

def data_processor_thread(line_queue, datastore, stop_event):
    """
    Dequeues lines, parses them, appends to datastore.
    """
    while not stop_event.is_set():
        try:
            # Process multiple lines if available
            for _ in range(100):
                line = line_queue.get_nowait()
                print(line)
                parsed = parse_line(line)
                print(parsed)
                if parsed:
                    datastore.append(*parsed)
        except queue.Empty:
            pass
        time.sleep(0.001)

# -----------------------
# Visual 1: Gantry
# -----------------------
class GantryVisualizer:
    """
    Simple 2-subplot figure:
      (1) G1 & G2 vs time
      (2) xVel & yVel vs time
    """
    def __init__(self, ds):
        self.ds = ds

        self.fig, (self.ax_motors, self.ax_vel) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.suptitle("Gantry Visualization")

        # Subplot: motor angles vs time
        self.ax_motors.set_title("Motor Angles (G1, G2)")
        self.ax_motors.set_xlabel("Time (s)")
        self.ax_motors.set_ylabel("Angle (units?)")
        self.line_g1, = self.ax_motors.plot([], [], 'b-', label="G1")
        self.line_g2, = self.ax_motors.plot([], [], 'r-', label="G2")
        self.ax_motors.legend()

        # Subplot: velocities vs time
        self.ax_vel.set_title("Gantry Velocity (xVel, yVel)")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (?)")
        self.line_xv, = self.ax_vel.plot([], [], 'g-', label="xVel")
        self.line_yv, = self.ax_vel.plot([], [], 'm-', label="yVel")
        self.ax_vel.legend()

    def update(self):
        ds = self.ds
        t  = ds.times

        self.line_g1.set_data(t, ds.G1)
        self.line_g2.set_data(t, ds.G2)
        self.line_xv.set_data(t, ds.xVel)
        self.line_yv.set_data(t, ds.yVel)

        # Autoscale
        if len(t) > 1:
            self.ax_motors.set_xlim(t[0], t[-1])
            self.ax_vel.set_xlim(t[0], t[-1])

        self.ax_motors.relim()
        self.ax_motors.autoscale_view()
        self.ax_vel.relim()
        self.ax_vel.autoscale_view()

        return []

# -----------------------
# Visual 2: Pendulum
# -----------------------
class PendulumVisualizer:
    """
    2-subplot figure:
      (1) E1 & E2 vs time
      (2) A tilt plot from (0,0) to (E2, E1)
    """
    def __init__(self, ds):
        self.ds = ds

        self.fig, (self.ax_e, self.ax_tilt) = plt.subplots(1, 2, figsize=(10, 4))
        self.fig.suptitle("Pendulum Visualization")

        # Subplot: E1/E2 vs time
        self.ax_e.set_title("Pendulum Angles (E1, E2) vs Time")
        self.ax_e.set_xlabel("Time (s)")
        self.ax_e.set_ylabel("Angle (units?)")
        self.line_e1, = self.ax_e.plot([], [], 'b-', label="E1")
        self.line_e2, = self.ax_e.plot([], [], 'r-', label="E2")
        self.ax_e.legend()

        # Subplot: Tilt
        self.ax_tilt.set_title("Tilt Plot (E2 vs E1)")
        self.ax_tilt.axhline(0, color='k', linestyle='--')
        self.ax_tilt.axvline(0, color='k', linestyle='--')
        self.ax_tilt.set_xlabel("E2")
        self.ax_tilt.set_ylabel("E1")
        self.line_tilt, = self.ax_tilt.plot([0,0],[0,0], 'g-o')

        self.ax_tilt.set_xlim(-500, 500)  # adjust as needed
        self.ax_tilt.set_ylim(-500, 500)

    def update(self):
        ds = self.ds
        t  = ds.times

        # Time-series
        self.line_e1.set_data(t, ds.E1)
        self.line_e2.set_data(t, ds.E2)
        if len(t) > 1:
            self.ax_e.set_xlim(t[0], t[-1])
        self.ax_e.relim()
        self.ax_e.autoscale_view()

        # Tilt plot: last E1/E2
        if len(ds.E1) > 0:
            e1_latest = ds.E1[-1]
            e2_latest = ds.E2[-1]
            self.line_tilt.set_data([0, e2_latest], [0, e1_latest])
            self.ax_tilt.relim()
            self.ax_tilt.autoscale_view()

        return []

# -----------------------
# Main
# -----------------------
def main():
    ds = DataStore(maxlen=MAX_POINTS)

    # Create visualizers
    gantry_viz   = GantryVisualizer(ds)
    pend_viz     = PendulumVisualizer(ds)

    # Open serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
    time.sleep(2)  # let port settle

    # Queues
    line_queue = queue.Queue()
    stop_event = threading.Event()

    # Threads
    t1 = threading.Thread(target=serial_reader_thread,
                          args=(ser, line_queue, stop_event),
                          daemon=True)
    t2 = threading.Thread(target=data_processor_thread,
                          args=(line_queue, ds, stop_event),
                          daemon=True)
    t1.start()
    t2.start()

    def update_gantry(frame):
        return gantry_viz.update()

    def update_pendulum(frame):
        return pend_viz.update()

    # Animate
    ani1 = animation.FuncAnimation(gantry_viz.fig, update_gantry,
                                   interval=UPDATE_INTERVAL_MS,
                                   blit=False)
    ani2 = animation.FuncAnimation(pend_viz.fig, update_pendulum,
                                   interval=UPDATE_INTERVAL_MS,
                                   blit=False)

    plt.show()

    # Clean up
    stop_event.set()
    ser.close()
    t1.join()
    t2.join()

if __name__ == "__main__":
    main()
