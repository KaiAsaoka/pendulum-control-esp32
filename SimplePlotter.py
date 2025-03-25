import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from collections import deque
import time
from threading import Lock

# Set up serial (adjust port name to suit)
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  # Reduced timeout
time.sleep(2)

# Initialize data structures with thread safety
max_points = 100
angles1 = deque([0] * max_points, maxlen=max_points)
angles2 = deque([0] * max_points, maxlen=max_points)
data_lock = Lock()
last_update_time = time.time()
update_interval = 0.05  # 50ms between updates

# Set up plot
fig, ax = plt.subplots()
ax.set_ylim(0, 16384)
ax.set_title("Real-Time Angle Encoder Data")
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Angle (deg)")
line1, = ax.plot(range(max_points), list(angles1), color='b', lw=2)
line2, = ax.plot(range(max_points), list(angles2), color='r', lw=2)

def read_serial_data():
    """Read and process serial data in a separate thread"""
    while True:
        try:
            if ser.in_waiting:
                data = ser.readline().decode().strip()
                with data_lock:
                    if data.startswith("Encoder 1 angle: "):
                        angle_str = data.replace("Encoder 1 angle: ", "")
                        angles1.append(float(angle_str))
                    if data.startswith("Encoder 2 angle: "):
                        angle_str = data.replace("Encoder 2 angle: ", "")
                        angles2.append(float(angle_str))
        except Exception as e:
            print(f"Error reading serial data: {e}")
        time.sleep(0.001)  # Small delay to prevent CPU hogging

def update(frame):
    """Update the plot with rate limiting"""
    global last_update_time
    current_time = time.time()
    
    # Rate limit updates
    if current_time - last_update_time < update_interval:
        return line1, line2
    
    try:
        with data_lock:
            # Update plot data
            line1.set_ydata(list(angles1))
            line2.set_ydata(list(angles2))
        last_update_time = current_time
        return line1, line2
    except Exception as e:
        print(f"Error updating plot: {e}")
        return line1, line2

# Start serial reading thread
import threading
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Start Animation with reduced update frequency
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.show()







"""
Real-time plot for given data over time

@param (float, float)[] data  data to be plotted
@param float timespan         maximum time interval to display
@param (float, float) ylim    y-axis limits for plot
"""
#def plot(data, timespan, ylim):
    