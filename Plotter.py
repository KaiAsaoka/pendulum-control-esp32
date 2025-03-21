import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from collections import deque
import time

#------------------------------------------------------------
## IMPORTANT!
## Serial input format: Each new entry is given a new line
## Line format goes: timestamp(ms) encoder[1...4](deg) positions[1...2](mm) angles[1...2](deg) ->
# -> coefficients_outer[Kp, Ki, Kd] coefficients_inner[Kp, Ki, Kd]
## All values separated by spaces
#------------------------------------------------------------

# Set up serial (adjust port name to suit)
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# Define number of entities
num_time = 1 #(obviously)
num_encoders = 4
num_positions = 2
num_angles = 2
num_coefficients_outer = 3
num_coefficients_inner = 3

num_values = num_time+num_encoders+num_positions+num_angles+num_coefficients_outer+num_coefficients_inner
value_names = ["time", "encL", "encR", "encX", "encY", "posX", "posY", "angX", "angY", "KpO", "KiO", "KdO", "KpI", "KiI", "KdI"]

# Initialize plots
fig_enc, fig_pos, fig_ang, fig_co, fig_ci = 0, 0, 0, 0, 0
ax_enc, ax_pos, ax_ang, ax_co, ax_ci = 0, 0, 0, 0, 0
enc_lines, pos_lines, ang_lines, co_lines, ci_lines = 0, 0, 0, 0, 0

# Initialize angles in double-ended queue (deque) with zeros
max_points = 100

values = [deque([0] * max_points, maxlen=max_points) for _ in range(num_values)]

# Other settings
TIMESPAN = 5000 # Number of ms to display on graph

# Set up plots
def setup_plots():
    # Set up plot (Encoders)
    fig_enc, ax_enc = plt.subplots()
    ax_enc.set_ylim(0, 360)
    ax_enc.set_title("Real-Time Angle Encoder Data")
    ax_enc.set_xlabel("Time (ms)")
    ax_enc.set_ylabel("Angle (deg)")
    enc_lines = [ax_enc.plot([], [], lw=2, label=f'Encoder {i+1}')[0] for i in range(num_encoders)]
    ax_enc.legend()

    #Set up plot (Position)
    fig_pos, ax_pos = plt.subplots()
    ax_pos.set_ylim(-500, 500)
    ax_pos.set_title("Position Tracking")
    ax_pos.set_xlabel("Time (ms)")
    ax_pos.set_ylabel("Position (mm)")
    pos_lines = [ax_pos.plot([], [], lw=2, label=f'Position {i+1}')[0] for i in range(num_positions)]
    ax_pos.legend()

    #Set up plot (Angle)
    fig_ang, ax_ang = plt.subplots()
    ax_ang.set_ylim(-180, 180)
    ax_ang.set_title("Angle Tracking")
    ax_ang.set_xlabel("Time (ms)")
    ax_ang.set_ylabel("Angle (deg)")
    ang_lines = [ax_ang.plot([], [], lw=2, label=f'Angle {i+1}')[0] for i in range(num_angles)]
    ax_ang.legend()

    #Set up plot (Outer Loop Coefficient Contributions)
    fig_co, ax_co = plt.subplots()
    #ax_co.set_ylim(0, 1)
    ax_co.set_title("Outer Loop Coefficient Contributions")
    ax_co.set_xlabel("Time (ms)")
    ax_co.set_ylabel("Value")
    co_lines = [ax_co.plot([], [], lw=2, label=f'Coefficient {i+1}')[0] for i in range(num_coefficients_outer)]
    ax_co.legend()

    #Set up plot (Inner Loop Coefficient Contributions)
    fig_ci, ax_ci = plt.subplots()
    #ax_ci.set_ylim(0, 1)
    ax_ci.set_title("Inner Loop Coefficient Contributions")
    ax_ci.set_xlabel("Time (ms)")
    ax_ci.set_ylabel("Value")
    ci_lines = [ax_ci.plot([], [], lw=2, label=f'Coefficient {i+1}')[0] for i in range(num_coefficients_inner)]
    ax_ci.legend()
setup_plots()

def update(frame):
    try:
        while ser.in_waiting:
            data = ser.readline().decode().strip()
            values = data.split()
            for i in range(num_values):
                values[i].append(float(values[i]))

        value_index = 1
        for i in range(num_encoders):
            enc_lines[i].set_xdata(list(values[0]))
            enc_lines[i].set_ydata(list(values[value_index+i]))
        ax_enc.set_xlim(max(values[0]) - 5000, max(values[0]))
        value_index += num_encoders
        for i in range(num_positions):
            pos_lines[i].set_xdata(list(values[0]))
            pos_lines[i].set_ydata(list(values[value_index+i]))
        ax_pos.set_xlim(max(values[0]) - 5000, max(values[0]))
        value_index += num_positions
        for i in range(num_angles):
            ang_lines[i].set_xdata(list(values[0]))
            ang_lines[i].set_ydata(list(values[value_index+i]))
        ax_ang.set_xlim(max(values[0]) - 5000, max(values[0]))
        value_index += num_angles
        for i in range(num_coefficients_outer):
            co_lines[i].set_xdata(list(values[0]))
            co_lines[i].set_ydata(list(values[value_index+i]))
        ax_co.set_xlim(max(values[0]) - 5000, max(values[0]))
        value_index += num_coefficients_outer
        for i in range(num_coefficients_inner):
            ci_lines[i].set_xdata(list(values[0]))
            ci_lines[i].set_ydata(list(values[value_index+i]))
        ax_ci.set_xlim(max(values[0]) - 5000, max(values[0]))

        return enc_lines + pos_lines + ang_lines + co_lines + ci_lines

    except Exception as e:
        print(f"Error: {e}")
        return enc_lines + pos_lines + ang_lines + co_lines + ci_lines,

# Start Animation
ani_enc = animation.FuncAnimation(fig_enc, update, interval=50, blit=True)
ani_pos = animation.FuncAnimation(fig_pos, update, interval=50, blit=True)
ani_ang = animation.FuncAnimation(fig_ang, update, interval=50, blit=True)
ani_co = animation.FuncAnimation(fig_co, update, interval=50, blit=True)
ani_ci = animation.FuncAnimation(fig_ci, update, interval=50, blit=True)

plt.show()







"""
Real-time plot for given data over time

@param (float, float)[] data  data to be plotted
@param float timespan         maximum time interval to display
@param (float, float) ylim    y-axis limits for plot
"""
#def plot(data, timespan, ylim):
    