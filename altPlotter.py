#!/usr/bin/env python3
"""
Updated version using the multi-thread + queue approach.

Data lines from serial are expected in this form:
  E1: <val>, E2: <val>, G1: <val>, G2: <val>, xVel: <val>, yVel: <val>

This code now:
  - Calibrates the pendulum angles so that the first reading is treated as upright (zero).
  - Calibrates the gantry velocities so that the initial xVel and yVel are treated as "no movement".
  - Visualizes the carriage position within a gantry space (32″ in x by 48″ in y) where
    the center is at (0,0). The carriage is shown as a 3″×5″ rectangle with velocity arrows.
  - Also shows motor angles and velocities versus time and pendulum angles (calibrated) versus time
    plus a tilt plot.

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
import matplotlib.patches as patches

#Attempt to import K values for PID (from main.cpp) MANUAL
PKP = 1
PKI = 0
PKD = 0
GKP = 1
GKI = 0
GKD = 0


# -------------
# User Config
# -------------
SERIAL_PORT = 'COM6'        # Adjust if needed
BAUD_RATE   = 115200
UPDATE_INTERVAL_MS = 10     # ~20 FPS
MAX_POINTS  = 1000          # store up to 1000 samples

# Gantry boundaries (in inches)
GANTRY_X_HALF = 16.0  # 32 inches wide
GANTRY_Y_HALF = 24.0  # 48 inches tall

#X and Y position scaling - inches to motor angles
X_scale = 675.0  # 1000 motor units = 1 inch
Y_scale = 575.0  # 1000 motor units = 1 inch

# --------------------------------------
# Data Store
# --------------------------------------
class DataStore:
    """
    Stores the time series for the six fields:
       E1, E2, G1, G2, xVel, yVel,
    plus computed carriage positions (in inches).

    For the pendulum, the first values for E1 and E2 are treated as the reference (upright),
    so that the calibrated values are relative to that.
    
    For the velocities, the first xVel and yVel are taken as the baseline (no movement).
    Subsequent velocity values are plotted as deviations from that baseline.
    """
    def __init__(self, maxlen=1000):
        self.maxlen = maxlen
        self.start_time = time.time()

        self.times = []
        self.E1    = []   # calibrated pendulum angle E1
        self.E2    = []   # calibrated pendulum angle E2
        self.G1    = []
        self.G2    = []
        self.xVel  = []   # calibrated gantry velocity (x)
        self.yVel  = []   # calibrated gantry velocity (y)
        self.Pex = []
        self.Pey = []
        self.Gex = []
        self.Gey = []
        
        # Add to the DataStore.__init__ method:
        # PID Components for X velocity controller
        self.xVelp = []  # P component for X velocity
        self.xVeli = []  # I component for X velocity 
        self.xVeld = []  # D component for X velocity

        # PID Components for Y velocity controller
        self.yVelp = []  # P component for Y velocity
        self.yVeli = []  # I component for Y velocity
        self.yVeld = []  # D component for Y velocity
        # Computed carriage position (in inches)
        self.carriageX = []
        self.carriageY = []

        # PID Components for angle setpoint controllers
        self.xspAngle = []  # X setpoint angle
        self.xspAp = []     # P component for X setpoint angle
        self.xspAi = []     # I component for X setpoint angle
        self.xspAd = []     # D component for X setpoint angle
        
        # PID Components for Y angle setpoint controller
        self.yspAngle = []  # Y setpoint angle
        self.yspAp = []     # P component for Y setpoint angle
        self.yspAi = []     # I component for Y setpoint angle 
        self.yspAd = []     # D component for Y setpoint angle

        # Calibration for pendulum:
        self._initialized_pendulum = False
        self._E1_center = 0.0
        self._E2_center = 0.0

        # Calibration for carriage (motor angles):
        self._initialized_carriage = False
        self._g1_center = 0.0
        self._g2_center = 0.0

        # Calibration for velocity:
        self._initialized_velocity = False
        self._xVel_base = 0.0
        self._yVel_base = 0.0

    def append(self, e1, e2, g1, g2, xv, yv, pex, pey, gex, gey, 
               xVelp, xVeli, xVeld, yVelp, yVeli, yVeld, 
               xspAngle, xspAp, xspAi, xspAd, yspAngle, yspAp, yspAi, yspAd):
        
        now = time.time() - self.start_time

        def store(lst, val):
            lst.append(val)
            if len(lst) > self.maxlen:
                lst.pop(0)

        store(self.times, now)

        store(self.Pex, pex)
        store(self.Pey, pey)
        store(self.Gex, gex)
        store(self.Gey, gey)
        
        store(self.xVelp, xVelp)
        store(self.xVeli, xVeli)
        store(self.xVeld, xVeld)
        store(self.yVelp, yVelp)
        store(self.yVeli, yVeli)
        store(self.yVeld, yVeld)
        
        store(self.xspAngle, xspAngle)
        store(self.xspAp, xspAp)
        store(self.xspAi, xspAi)
        store(self.xspAd, xspAd)

        
        store(self.yspAngle, yspAngle)
        store(self.yspAp, yspAp)
        store(self.yspAi, yspAi)
        store(self.yspAd, yspAd)
        
        # Calibrate pendulum angles:
        if not self._initialized_pendulum:
            self._E1_center = 0
            self._E2_center = 0
            self._initialized_pendulum = True
        e1_cal = e1 - self._E1_center
        e2_cal = e2 - self._E2_center
        store(self.E1, e1_cal)
        store(self.E2, e2_cal)

        # Store raw motor angles:
        store(self.G1, g1)
        store(self.G2, g2)

        # Calibrate velocities:
        if not self._initialized_velocity:
            self._xVel_base = 0
            self._yVel_base = 0
            self._initialized_velocity = True
        xv_cal = xv - self._xVel_base
        yv_cal = yv - self._yVel_base
        store(self.xVel, xv_cal)
        store(self.yVel, yv_cal)

        # Compute carriage position from motor angles (placeholder logic)
        x_in, y_in = self.calculate_carriage_position(g1, g2)
        store(self.carriageX, x_in)
        store(self.carriageY, y_in)

    def calculate_carriage_position(self, g1, g2):
        """
        Converts motor angles (g1, g2) into carriage X, Y (in inches),
        with (0,0) at the center of the gantry.

        Placeholder implementation:
          - The first reading of G1 and G2 is treated as the center.
          - Then a naive linear mapping is applied.
        """
        if not self._initialized_carriage:
            self._g1_center = g1
            self._g2_center = g2
            self._initialized_carriage = True
            return 0.0, 0.0  # center

        dg1 = g1 - self._g1_center
        dg2 = g2 - self._g2_center
        # Motor to xy displacement scaling
        x_in = dg1 / X_scale
        y_in = dg2 / Y_scale

        # Bound the results within the gantry limits.
        x_in = max(-GANTRY_X_HALF, min(x_in, GANTRY_X_HALF))
        y_in = max(-GANTRY_Y_HALF, min(y_in, GANTRY_Y_HALF))
        return (-x_in, -y_in)

# --------------------------------------
# Parsing
# --------------------------------------
def parse_line(line):
    """
    Expects a line in the form:
      E1: <val>, E2: <val>, G1: <val>, G2: <val>, xVel: <val>, yVel: <val>
    Returns (E1, E2, G1, G2, xVel, yVel) as floats or None on failure.
    """
    line = line.strip()
    if not line:
        return None

    parts = line.split(',')
    if len(parts) < 16:
        return None

    try:
        e1_val = float(parts[0].split(':')[1].strip())
        e2_val = float(parts[1].split(':')[1].strip())
        g1_val = float(parts[2].split(':')[1].strip())
        g2_val = float(parts[3].split(':')[1].strip())
        xv_val = float(parts[4].split(':')[1].strip())
        yv_val = float(parts[5].split(':')[1].strip())
        pex_val = float(parts[6].split(':')[1].strip())
        pey_val = float(parts[7].split(':')[1].strip())
        gex_val = float(parts[8].split(':')[1].strip())
        gey_val = float(parts[9].split(':')[1].strip())
        xVelp_val = float(parts[10].split(':')[1].strip())
        xVeli_val = float(parts[11].split(':')[1].strip())
        xVeld_val = float(parts[12].split(':')[1].strip())
        yVelp_val = float(parts[13].split(':')[1].strip())
        yVeli_val = float(parts[14].split(':')[1].strip())
        yVeld_val = float(parts[15].split(':')[1].strip())
        setPointAngle1 = float(parts[16].split(':')[1].strip())
        angle1p = float(parts[17].split(':')[1].strip())
        angle1i = float(parts[18].split(':')[1].strip())
        angle1d = float(parts[19].split(':')[1].strip())
        setPointAngle2 = float(parts[20].split(':')[1].strip())
        angle2p = float(parts[21].split(':')[1].strip())
        angle2i = float(parts[22].split(':')[1].strip())
        angle2d = float(parts[23].split(':')[1].strip())


        return (e1_val, e2_val, g1_val, g2_val, xv_val, yv_val, 
                pex_val, pey_val, gex_val, gey_val,
                xVelp_val, xVeli_val, xVeld_val, yVelp_val, yVeli_val, yVeld_val, 
                setPointAngle1, angle1p, angle1i, angle1d, setPointAngle2, angle2p, angle2i, angle2d)
    except:
        return None

# --------------------------------------
# Background Threads
# --------------------------------------
def serial_reader_thread(ser, line_queue, stop_event):
    """
    Continuously reads lines from the serial port and places them in line_queue.
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
    Dequeues lines from the queue, parses them, and appends data to the datastore.
    """
    while not stop_event.is_set():
        try:
            for _ in range(100):
                line = line_queue.get_nowait()
                parsed = parse_line(line)
                print(parsed)
                if parsed:
                    datastore.append(*parsed)
        except queue.Empty:
            pass
        time.sleep(0.001)

# --------------------------------------
# Visualization: Carriage Position (Separate Figure)
# --------------------------------------
class CarriagePositionVisualizer:
    """
    Visualizes the carriage’s current position within the gantry space.
    It shows:
      - A rectangle representing the carriage (3″ × 5″) whose center is its position.
      - An arrow vector representing the velocity (xVel, yVel) anchored at the carriage center.
      - The gantry boundaries: x from -16 to +16 inches, y from -24 to +24 inches.
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, self.ax = plt.subplots(figsize=(8,8))
        self.ax.set_title("Carriage Position in Gantry Space")
        self.ax.set_xlabel("X (inches)")
        self.ax.set_ylabel("Y (inches)")
        self.ax.set_xlim(-GANTRY_X_HALF, GANTRY_X_HALF)
        self.ax.set_ylim(-GANTRY_Y_HALF, GANTRY_Y_HALF)
        self.ax.set_aspect('equal', adjustable='box')

        # Draw boundary box for gantry space.
        self.ax.plot([-GANTRY_X_HALF, GANTRY_X_HALF, GANTRY_X_HALF, -GANTRY_X_HALF, -GANTRY_X_HALF],
                     [-GANTRY_Y_HALF, -GANTRY_Y_HALF, GANTRY_Y_HALF, GANTRY_Y_HALF, -GANTRY_Y_HALF],
                     'k--', lw=1)

        # Carriage dimensions in inches.
        self.carriage_width = 3.0
        self.carriage_height = 5.0
        # Create a rectangle patch for the carriage, initially centered at (0,0).
        self.carriage_rect = patches.Rectangle((-self.carriage_width/2, -self.carriage_height/2),
                                                 self.carriage_width,
                                                 self.carriage_height,
                                                 linewidth=2, edgecolor='r', facecolor='none')
        self.ax.add_patch(self.carriage_rect)

        # Placeholder for the velocity arrow.
        self.vel_arrow = None
        self.vel_scale = 0.1  # scaling factor for velocity vector

        # Add PID values as a note outside the plot area
        pid_text = (
            f"Pendulum PID:\n[PKP: {PKP}, PKI: {PKI}, PKD: {PKD}]\n\n"
            f"Gantry PID:\n[GKP: {GKP}, GKI: {GKI}, GKD: {GKD}]"
        )
        self.fig.text(0.02, 0.95, pid_text, fontsize=10, verticalalignment='top', horizontalalignment='left')


    def update(self):
        ds = self.ds
        if len(ds.carriageX) > 0:
            x = ds.carriageX[-1]
            y = ds.carriageY[-1]
            # Update carriage rectangle so its center is at (x,y)
            self.carriage_rect.set_xy((x - self.carriage_width/2, y - self.carriage_height/2))
            # Remove previous velocity arrow if it exists.
            if self.vel_arrow is not None:
                self.vel_arrow.remove()
                self.vel_arrow = None
            # Draw new velocity arrow from carriage center.
            if len(ds.xVel) > 0 and len(ds.yVel) > 0:
                xv = ds.xVel[-1]
                yv = ds.yVel[-1]
                dx = xv * self.vel_scale
                dy = yv * self.vel_scale
                self.vel_arrow = self.ax.arrow(x, y, dx, dy,
                                               head_width=1.0, head_length=1.0,
                                               fc='b', ec='b')
        return []

# --------------------------------------
# Visualization: Gantry (Motor Angles & Velocities)
# --------------------------------------
class GantryVisualizer:
    """
    Visualizes motor angles (G1, G2) and velocities (xVel, yVel) versus time.
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, (self.ax_motors, self.ax_vel) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.suptitle("Gantry: Motor Angles and Velocities")

        self.ax_motors.set_title("Motor Angles (G1, G2)")
        self.ax_motors.set_xlabel("Time (s)")
        self.ax_motors.set_ylabel("Angle (units)")
        self.line_g1, = self.ax_motors.plot([], [], 'b-', label="G1")
        self.line_g2, = self.ax_motors.plot([], [], 'r-', label="G2")
        self.ax_motors.legend()

        self.ax_vel.set_title("Gantry Velocity (xVel, yVel)")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (units)")
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
        if len(t) > 1:
            self.ax_motors.set_xlim(t[0], t[-1])
            self.ax_vel.set_xlim(t[0], t[-1])
        self.ax_motors.relim()
        self.ax_motors.autoscale_view()
        self.ax_vel.relim()
        self.ax_vel.autoscale_view()
        return []

# --------------------------------------
# Visualization: Pendulum
# --------------------------------------
class PendulumVisualizer:
    """
    Visualizes pendulum angles (E1, E2) versus time and a tilt plot (E2 vs E1).
    (Pendulum angles are calibrated so that 0 is the upright position.)
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, (self.ax_e, self.ax_tilt) = plt.subplots(1, 2, figsize=(10, 4))
        self.fig.suptitle("Pendulum Visualization")

        self.ax_e.set_title("Pendulum Angles vs Time")
        self.ax_e.set_xlabel("Time (s)")
        self.ax_e.set_ylabel("Angle (units)")
        self.line_e1, = self.ax_e.plot([], [], 'b-', label="E1")
        self.line_e2, = self.ax_e.plot([], [], 'r-', label="E2")
        self.ax_e.legend()

        self.ax_tilt.set_title("Pendulum Tilt (E2 vs E1)")
        self.ax_tilt.axhline(0, color='k', linestyle='--')
        self.ax_tilt.axvline(0, color='k', linestyle='--')
        self.ax_tilt.set_xlabel("E2")
        self.ax_tilt.set_ylabel("E1")
        self.line_tilt, = self.ax_tilt.plot([0,0], [0,0], 'g-o')
        self.ax_tilt.set_xlim(-500, 500)
        self.ax_tilt.set_ylim(-500, 500)

    def update(self):
        ds = self.ds
        t  = ds.times

        self.line_e1.set_data(t, ds.E1)
        self.line_e2.set_data(t, ds.E2)
        if len(t) > 1:
            self.ax_e.set_xlim(t[0], t[-1])
        self.ax_e.relim()
        self.ax_e.autoscale_view()

        if len(ds.E1) > 0:
            e1_latest = ds.E1[-1]
            e2_latest = ds.E2[-1]
            self.line_tilt.set_data([0, -e1_latest], [0, -e2_latest])
            self.ax_tilt.relim()
            self.ax_tilt.autoscale_view()

        return []

# --------------------------------------
# Visualization: Error
# --------------------------------------
class ErrorVisualizer:
    """
    Visualizes motor angles (G1, G2) and velocities (xVel, yVel) versus time.
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, (self.ax_perr, self.ax_gerr) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.suptitle("Error: Real-Time Angle Encoder Data")

        self.ax_perr.set_title("Angle Error")
        self.ax_perr.set_xlabel("Time (s)")
        self.ax_perr.set_ylabel("Angle (units)")
        self.line_e1, = self.ax_perr.plot([], [], 'b-', label="X Angle Error")
        self.line_e2, = self.ax_perr.plot([], [], 'r-', label="Y Angle Error")
        self.ax_perr.axhline(0, color='black', linestyle='--', linewidth=1)
        self.ax_perr.legend()

        self.ax_gerr.set_title("Position Error")
        self.ax_gerr.set_xlabel("Time (s)")
        self.ax_gerr.set_ylabel("Position (units)")
        self.line_g1, = self.ax_gerr.plot([], [], 'g-', label="X Pos Error")
        self.line_g2, = self.ax_gerr.plot([], [], 'm-', label="Y Pos Error")
        self.ax_gerr.axhline(0, color='black', linestyle='--', linewidth=1)
        self.ax_gerr.legend()

    def update(self):
        ds = self.ds
        t  = ds.times

        self.line_e1.set_data(t, ds.Pex)
        self.line_e2.set_data(t, ds.Pey)
        self.line_g1.set_data(t, ds.Gex)
        self.line_g2.set_data(t, ds.Gey)
        if len(t) > 1:
            self.ax_perr.set_xlim(t[0], t[-1])
            self.ax_gerr.set_xlim(t[0], t[-1])
        self.ax_perr.relim()
        self.ax_perr.autoscale_view()
        self.ax_gerr.relim()
        self.ax_gerr.autoscale_view()
        return []

# --------------------------------------
# Visualization: Error
# --------------------------------------
class AngleErrorVisualizer:
    """
    Visualizes motor angles (G1, G2) and velocities (xVel, yVel) versus time.
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, (self.ax_pex, self.ax_pey) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.suptitle("Error: Real-Time Angle Encoder Data")

        self.ax_pex.set_title("Angle Error X")
        self.ax_pex.set_xlabel("Time (s)")
        self.ax_pex.set_ylabel("Angle (units)")
        self.line_e1, = self.ax_pex.plot([], [], 'b-', label="X Angle Error")
        self.line_e2, = self.ax_pex.plot([], [], 'r-', label="X PID Output")
        self.ax_pex.axhline(0, color='black', linestyle='--', linewidth=1)
        self.ax_pex.legend()

        self.ax_pey.set_title("Angle Error Y")
        self.ax_pey.set_xlabel("Time (s)")
        self.ax_pey.set_ylabel("Angle (units)")
        self.line_g1, = self.ax_pey.plot([], [], 'g-', label="Y Angle Error")
        self.line_g2, = self.ax_pey.plot([], [], 'm-', label="Y PID Output")
        self.ax_pey.axhline(0, color='black', linestyle='--', linewidth=1)
        self.ax_pey.legend()

    def update(self):
        ds = self.ds
        t  = ds.times

        self.line_e1.set_data(t, ds.Pex)
        self.line_e2.set_data(t, ds.xVel)
        self.line_g1.set_data(t, ds.Pey)
        self.line_g2.set_data(t, ds.yVel)
        if len(t) > 1:
            self.ax_pex.set_xlim(t[0], t[-1])
            self.ax_pey.set_xlim(t[0], t[-1])
        self.ax_pex.relim()
        self.ax_pex.autoscale_view()
        self.ax_pey.relim()
        self.ax_pey.autoscale_view()
        return []
# --------------------------------------
# Visualization: PID Components
# --------------------------------------
class VelocityPIDVisualizer:
    """
    Visualizes the individual PID components (P, I, D) for both velocity controllers.
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, (self.ax_xvel, self.ax_yvel) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle("Velocity PID Controller Components")

        # X Velocity Controller Components
        self.ax_xvel.set_title("X Velocity PID Components")
        self.ax_xvel.set_xlabel("Time (s)")
        self.ax_xvel.set_ylabel("Value")
        self.line_xvelp, = self.ax_xvel.plot([], [], 'r-', label="P")
        self.line_xveli, = self.ax_xvel.plot([], [], 'g-', label="I")
        # self.line_xveld, = self.ax_xvel.plot([], [], 'b-', label="D")
        self.line_xvele, = self.ax_xvel.plot([], [], 'k--', label="Error")
        self.line_xvelout, = self.ax_xvel.plot([], [], 'm-', linewidth=2, label="Output")
        self.ax_xvel.axhline(0, color='black', linestyle='--', linewidth=0.5)
        self.ax_xvel.legend()
        
        # Y Velocity Controller Components
        self.ax_yvel.set_title("Y Velocity PID Components")
        self.ax_yvel.set_xlabel("Time (s)")
        self.ax_yvel.set_ylabel("Value")
        self.line_yvelp, = self.ax_yvel.plot([], [], 'r-', label="P")
        self.line_yveli, = self.ax_yvel.plot([], [], 'g-', label="I")
        # self.line_yveld, = self.ax_yvel.plot([], [], 'b-', label="D")
        self.line_yvele, = self.ax_yvel.plot([], [], 'k--', label="Error")
        self.line_yvelout, = self.ax_yvel.plot([], [], 'm-', linewidth=2, label="Output")
        self.ax_yvel.axhline(0, color='black', linestyle='--', linewidth=0.5)
        self.ax_yvel.legend()

    def update(self):
        ds = self.ds
        t = ds.times
        
        # Update X Velocity controller data
        self.line_xvelp.set_data(t, ds.xVelp)
        self.line_xveli.set_data(t, ds.xVeli)
        # self.line_xveld.set_data(t, ds.xVeld)
        self.line_xvele.set_data(t, ds.Pex)  # Using pendulum x error
        self.line_xvelout.set_data(t, ds.xVel)
        
        # Update Y Velocity controller data
        self.line_yvelp.set_data(t, ds.yVelp)
        self.line_yveli.set_data(t, ds.yVeli)
        # self.line_yveld.set_data(t, ds.yVeld)
        self.line_yvele.set_data(t, ds.Pey)  # Using pendulum y error
        self.line_yvelout.set_data(t, ds.yVel)
        
        if len(t) > 1:
            self.ax_xvel.set_xlim(t[0], t[-1])
            self.ax_yvel.set_xlim(t[0], t[-1])
            
        self.ax_xvel.relim()
        self.ax_xvel.autoscale_view()
        self.ax_yvel.relim()
        self.ax_yvel.autoscale_view()
        
        return []
    
    
# --------------------------------------
# Visualization: PID Components
# --------------------------------------
class AngleSetpointPIDVisualizer:
    """
    Visualizes the individual PID components (P, I, D) for angle setpoint controllers.
    """
    def __init__(self, ds):
        self.ds = ds
        self.fig, (self.ax_xangle, self.ax_yangle) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle("Angle Setpoint PID Controller Components")

        # X Angle Setpoint Controller Components
        self.ax_xangle.set_title("X Angle Setpoint PID Components")
        self.ax_xangle.set_xlabel("Time (s)")
        self.ax_xangle.set_ylabel("Value")
        self.line_xsp, = self.ax_xangle.plot([], [], 'm-', label="Setpoint")
        self.line_xspap, = self.ax_xangle.plot([], [], 'r-', label="P")
        self.line_xspai, = self.ax_xangle.plot([], [], 'g-', label="I")
        self.line_xspad, = self.ax_xangle.plot([], [], 'b-', label="D")
        # self.line_xangle_out, = self.ax_xangle.plot([], [], 'k--', linewidth=2, label="Position Error X")
        self.ax_xangle.axhline(0, color='black', linestyle='--', linewidth=0.5)
        self.ax_xangle.legend()
        
        # Y Angle Setpoint Controller Components
        self.ax_yangle.set_title("Y Angle Setpoint PID Components")
        self.ax_yangle.set_xlabel("Time (s)")
        self.ax_yangle.set_ylabel("Value")
        self.line_ysp, = self.ax_yangle.plot([], [], 'm-', label="Setpoint")
        self.line_yspap, = self.ax_yangle.plot([], [], 'r-', label="P")
        self.line_yspai, = self.ax_yangle.plot([], [], 'g-', label="I")
        self.line_yspad, = self.ax_yangle.plot([], [], 'b-', label="D")
        # self.line_yangle_out, = self.ax_yangle.plot([], [], 'k--', linewidth=2, label="Position Error Y")
        self.ax_yangle.axhline(0, color='black', linestyle='--', linewidth=0.5)
        self.ax_yangle.legend()

    def update(self):
        ds = self.ds
        t = ds.times
        
        # Update X Angle Setpoint controller data
        self.line_xsp.set_data(t, ds.xspAngle)
        self.line_xspap.set_data(t, ds.xspAp)
        self.line_xspai.set_data(t, ds.xspAi)
        self.line_xspad.set_data(t, ds.xspAd)
        # self.line_xangle_out.set_data(t, ds.Gex)  # Position error X
        
        # Update Y Angle Setpoint controller data
        self.line_ysp.set_data(t, ds.yspAngle)
        self.line_yspap.set_data(t, ds.yspAp)
        self.line_yspai.set_data(t, ds.yspAi)
        self.line_yspad.set_data(t, ds.yspAd)
        # self.line_yangle_out.set_data(t, ds.Gey)  # Position error Y
        
        if len(t) > 1:
            self.ax_xangle.set_xlim(t[0], t[-1])
            self.ax_yangle.set_xlim(t[0], t[-1])
            
        self.ax_xangle.relim()
        self.ax_xangle.autoscale_view()
        self.ax_yangle.relim()
        self.ax_yangle.autoscale_view()
        
        return []
    
    
# --------------------------------------
# Main
# --------------------------------------
def main():
    ds = DataStore(maxlen=MAX_POINTS)

    # Create visualizers.
    # gantry_vis = GantryVisualizer(ds)
    # pend_vis   = PendulumVisualizer(ds)
    # carriage_vis = CarriagePositionVisualizer(ds)  # Separate figure for carriage
    #error_vis = AngleErrorVisualizer(ds)
    velocity_vis = VelocityPIDVisualizer(ds)
    gantry_err_vis = AngleSetpointPIDVisualizer(ds)

    # Open serial port.
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
    time.sleep(0.001)

    line_queue = queue.Queue()
    stop_event = threading.Event()

    t1 = threading.Thread(target=serial_reader_thread,
                          args=(ser, line_queue, stop_event),
                          daemon=True)
    t2 = threading.Thread(target=data_processor_thread,
                          args=(line_queue, ds, stop_event),
                          daemon=True)
    t1.start()
    t2.start()

    # def update_gantry(frame):
    #     return gantry_vis.update()

    # def update_pendulum(frame):
    #     return pend_vis.update()

    # def update_carriage(frame):
    #     return carriage_vis.update()
    
    # def update_error(frame):
    #     return error_vis.update()
    def update_velocity(frame):
        return velocity_vis.update()
    
    def update_gantry_err(frame):
        return gantry_err_vis.update()

    # ani1 = animation.FuncAnimation(gantry_vis.fig, update_gantry,
    #                                interval=UPDATE_INTERVAL_MS, blit=False)
    # ani2 = animation.FuncAnimation(pend_vis.fig, update_pendulum,
    #                                interval=UPDATE_INTERVAL_MS, blit=False)
    # ani3 = animation.FuncAnimation(carriage_vis.fig, update_carriage,
    #                                interval=UPDATE_INTERVAL_MS, blit=False)
    # ani4 = animation.FuncAnimation(error_vis.fig, update_error,
    #                                interval=UPDATE_INTERVAL_MS, blit=False)
    ani5 = animation.FuncAnimation(velocity_vis.fig, update_velocity,
                                interval=UPDATE_INTERVAL_MS, blit=False)
    ani6 = animation.FuncAnimation(gantry_err_vis.fig, update_gantry_err,
                                interval=UPDATE_INTERVAL_MS, blit=False)

    plt.show()

    stop_event.set()
    ser.close()
    t1.join()
    t2.join()

if __name__ == "__main__":
    main()
