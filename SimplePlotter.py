import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from collections import deque
import time
from threading import Lock
import queue
from array import array

# Set up serial with optimized buffer
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001, write_timeout=1)
ser.reset_input_buffer()
time.sleep(2)

# Initialize data structures with thread safety
max_points = 100
# Use numpy arrays for better performance
angles1 = np.zeros(max_points)
angles2 = np.zeros(max_points)
angles3 = np.zeros(max_points)  # G1
angles4 = np.zeros(max_points)  # G2
data_lock = Lock()
last_update_time = time.time()
update_interval = 0.05  # 50ms between updates

# Create a thread-safe queue for data processing
data_queue = queue.Queue(maxsize=1000)

# Set up plots with optimized settings
plt.style.use('fast')  # Use fast style for better performance
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle("Real-Time Angle Encoder Data")

# First subplot for E1, E2
ax1.set_ylim(0, 16384)
ax1.set_title("Encoders E1, E2")
ax1.set_xlabel("Time (samples)")
ax1.set_ylabel("Angle (deg)")
line1, = ax1.plot(angles1, color='b', lw=2)
line2, = ax1.plot(angles2, color='r', lw=2)

# Second subplot for G1, G2
ax2.set_ylim(0, 16384)
ax2.set_title("Encoders G1, G2")
ax2.set_xlabel("Time (samples)")
ax2.set_ylabel("Angle (deg)")
line3, = ax2.plot(angles3, color='g', lw=2)
line4, = ax2.plot(angles4, color='m', lw=2)

def process_serial_data():
    """Process serial data in chunks for better performance"""
    buffer = ""
    while True:
        try:
            if ser.in_waiting:
                # Read in chunks for better performance
                chunk = ser.read(ser.in_waiting).decode()
                buffer += chunk
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        data_queue.put(line)
        except Exception as e:
            print(f"Error reading serial data: {e}")
        time.sleep(0.001)

def update_data():
    """Update data arrays from queue"""
    while True:
        try:
            # Process multiple items at once for better performance
            for _ in range(100):  # Process up to 100 items per iteration
                try:
                    data = data_queue.get_nowait()
                    if data.startswith("E1: "):
                        angles1[:-1] = angles1[1:]
                        angles1[-1] = float(data[4:])
                    elif data.startswith("E2: "):
                        angles2[:-1] = angles2[1:]
                        angles2[-1] = float(data[4:])
                    elif data.startswith("G1: "):
                        angles3[:-1] = angles3[1:]
                        angles3[-1] = float(data[4:])
                    elif data.startswith("G2: "):
                        angles4[:-1] = angles4[1:]
                        angles4[-1] = float(data[4:])
                except queue.Empty:
                    break
        except Exception as e:
            print(f"Error updating data: {e}")
        time.sleep(0.001)

def update(frame):
    """Update the plot with rate limiting"""
    global last_update_time
    current_time = time.time()
    
    # Rate limit updates
    if current_time - last_update_time < update_interval:
        return line1, line2, line3, line4
    
    try:
        # Update plot data directly from numpy arrays
        line1.set_ydata(angles1)
        line2.set_ydata(angles2)
        line3.set_ydata(angles3)
        line4.set_ydata(angles4)
        last_update_time = current_time
        return line1, line2, line3, line4
    except Exception as e:
        print(f"Error updating plot: {e}")
        return line1, line2, line3, line4

# Start processing threads
import threading
serial_thread = threading.Thread(target=process_serial_data, daemon=True)
data_thread = threading.Thread(target=update_data, daemon=True)
serial_thread.start()
data_thread.start()

# Start Animation with optimized settings
ani = animation.FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()







"""
Real-time plot for given data over time

@param (float, float)[] data  data to be plotted
@param float timespan         maximum time interval to display
@param (float, float) ylim    y-axis limits for plot
"""
#def plot(data, timespan, ylim):
    