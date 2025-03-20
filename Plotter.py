import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from collections import deque
import time

# Set up serial (adjust port name to suit)
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# Initialize angles in double-ended queue (deque) with zeros
max_points = 100
angles = deque([0] * max_points, maxlen=max_points)

# Set up plot
fig, ax = plt.subplots()
ax.set_ylim(0, 360)
ax.set_title("Real-Time Angle Encoder Data")
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Angle (deg)")
line, = ax.plot(range(max_points), angles, color='b', lw=2)

def update(frame):
    try:
        while ser.in_waiting:
            data = ser.readline().decode().strip()
            angles.append(float(data))
        line.set_ydata(list(angles))  # Update plot
        return line,
    except Exception as e:
        print(f"Error: {e}")
        return line,

# Start Animation
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.show()







"""
Real-time plot for given data over time

@param (float, float)[] data  data to be plotted
@param float timespan         maximum time interval to display
@param (float, float) ylim    y-axis limits for plot
"""
#def plot(data, timespan, ylim):
    