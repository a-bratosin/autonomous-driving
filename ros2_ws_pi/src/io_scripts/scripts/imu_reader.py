from smbus2 import SMBus
import time


IMU_ADDRESS = 0x6a
WHO_AM_I_REG = 0x0F

CTRL1_XL = 0x10  # Accelerometer control
CTRL2_G = 0x11   # Gyroscope control
OUTX_L_G = 0x22  # Gyro data start
OUTX_L_A = 0x28  # Accel data start

# Sensitivity values for   2g and 250dps
ACCEL_SENSITIVITY = 0.061 / 1000  # mg/LSB  ^f^r g  ^f^r m/s  
ACCEL_SENSITIVITY *= 9.81  # Convert to m/s  
GYRO_SENSITIVITY = 8.75 / 1000  # mdps/LSB  ^f^r dps  ^f^r rad/s
GYRO_SENSITIVITY *= 0.017453293  # Convert to rad/s




# check if correct I2C device
def init_imu():
    bus =  SMBus()
    bus.open(1)


    if bus.read_byte_data(IMU_ADDRESS, WHO_AM_I_REG) not in [0x6b, 0x6c]:
        print("Warning: unexpected WHO_AM_I value!")


    bus.write_byte_data(IMU_ADDRESS, CTRL1_XL, 0X40)
    bus.write_byte_data(IMU_ADDRESS, CTRL2_G, 0X40)

    # timp pt stabilizare
    time.sleep(0.1)

    print(" ^|^s LSM6DSO configured successfully!\n")
    print("Reading sensor data (Press Ctrl+C to stop)...\n")
    return bus


def read_sensor_data(bus):
    """Read and convert accelerometer and gyroscope data"""
    # Read accelerometer (6 bytes: X, Y, Z as 16-bit values)

    accel_data = bus.read_i2c_block_data(IMU_ADDRESS, OUTX_L_A, 6)
    #print(accel_data)

    ax_raw = int.from_bytes(bytes(accel_data[0:2]), byteorder='little', signed=True)
    ay_raw = int.from_bytes(bytes(accel_data[2:4]), byteorder='little', signed=True)
    az_raw = int.from_bytes(bytes(accel_data[4:6]), byteorder='little', signed=True)
    
    # Read gyroscope (6 bytes: X, Y, Z as 16-bit values)
    gyro_data = bus.read_i2c_block_data(IMU_ADDRESS, OUTX_L_G, 6)
    
    #valorile sunt date de IMU ca variabile de tip signed short int; trebuie să le prelucrez pt fiecare în parte pt a ajunge la forma aceasta
    gx_raw = int.from_bytes(bytes(gyro_data[0:2]), byteorder='little', signed=True)
    gy_raw = int.from_bytes(bytes(gyro_data[2:4]), byteorder='little', signed=True)
    gz_raw = int.from_bytes(bytes(gyro_data[4:6]), byteorder='little', signed=True)
    #print(f"{hex(gx_raw)}, {hex(gy_raw)}, {hex(gz_raw)}")

    # Convert to physical units
    accel = (
        round(ax_raw * ACCEL_SENSITIVITY,3),
        round(ay_raw * ACCEL_SENSITIVITY,3),
        round(az_raw * ACCEL_SENSITIVITY,3)
    )
    
    gyro = (
        round(gx_raw * GYRO_SENSITIVITY,3),
        round(gy_raw * GYRO_SENSITIVITY,3),
        round(gz_raw * GYRO_SENSITIVITY,3)
    )
    
    return accel, gyro




if __name__=="__main__":
    bus = init_imu()
    try:
        while True:
            accel, gyro = read_sensor_data(bus)

            print(f"Accel: X={accel[0]} Y={accel[1]} Z={accel[2]} m/s  ")
            print(f"Gyro:  X={gyro[0]} Y={gyro[1]} Z={gyro[2]} rad/s")
            print("-" * 50)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")

