import smbus2
import time

# MPU6050 Default I2C address (may change)
DEVICE_ADDR = 0x68

# Register Map
PWR_MGMT_1   = 0x6B
GYRO_XOUT_H  = 0x43

# Gyroscope Sensitivity Scale Factor 
# (For the default range of +/- 250 degrees/sec)
GYRO_SENSITIVITY = 131.0 

bus = smbus2.SMBus(1) # Use I2C bus 1

def init_imu():
    # Wake up the MPU-6050 (it starts in sleep mode)
    bus.write_byte_data(DEVICE_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    # Accel and Gyro data are 16-bit (2 bytes)
    high = bus.read_byte_data(DEVICE_ADDR, addr)
    low = bus.read_byte_data(DEVICE_ADDR, addr+1)
    
    # Combine high and low bytes
    value = ((high << 8) | low)
    
    # Convert to signed 16-bit integer
    if value > 32768:
        value = value - 65536
    return value

init_imu()

print("Reading Gyroscope Data (degrees/sec)...")

try:
    while True:
        # Read raw gyroscope bits
        raw_x = read_raw_data(GYRO_XOUT_H)
        raw_y = read_raw_data(GYRO_XOUT_H + 2)
        raw_z = read_raw_data(GYRO_XOUT_H + 4)

        # Convert raw bits to Angular Velocity (dps)
        # Formula: omega = Raw Value / Sensitivity
        gyro_x = raw_x / GYRO_SENSITIVITY
        gyro_y = raw_y / GYRO_SENSITIVITY
        gyro_z = raw_z / GYRO_SENSITIVITY

        print(f"X: {gyro_x:6.2f} | Y: {gyro_y:6.2f} | Z: {gyro_z:6.2f} °/s")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")