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
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# Define number of entities
num_time = 1 #(obviously)
num_encoders = 1

num_values = num_time+num_encoders

# Initialize angles in double-ended queue (deque) with zeros
max_points = 100

values = [deque([0] * max_points, maxlen=max_points) for _ in range(num_values)]

# Other settings
TIMESPAN = 5000 # Number of ms to display on graph

# Set up plot (Encoders)
fig_enc, ax_enc = plt.subplots()
ax_enc.set_ylim(0, 360)
ax_enc.set_title("Real-Time Angle Encoder Data")
ax_enc.set_xlabel("Time (ms)")
ax_enc.set_ylabel("Angle (deg)")
enc_lines = [ax_enc.plot([], [], lw=2, label=f'Encoder {i+1}')[0] for i in range(num_encoders)]
ax_enc.legend()

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

        return enc_lines

    except Exception as e:
        print(f"Error: {e}")
        return enc_lines

# Start Animation
ani_enc = animation.FuncAnimation(fig_enc, update, interval=50, blit=True)

plt.show()







"""
Real-time plot for given data over time

@param (float, float)[] data  data to be plotted
@param float timespan         maximum time interval to display
@param (float, float) ylim    y-axis limits for plot
"""
#def plot(data, timespan, ylim):
    