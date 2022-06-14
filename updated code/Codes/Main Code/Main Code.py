

import RPi.GPIO as GPIO
from Kalman_Filter import KalmanAngle
import smbus			# Import SMBus module of I2C
import time
import math
from simple_pid import PID

GPIO.setwarnings(False)          # Disable warnings
GPIO.setmode(GPIO.BOARD)         # Set pin numbering system

# Pins
ANALOG_L = 33                    # Left motor
DIR_L = 31
BRAKE_L = 35
STOP_L = 37

ANALOG_R = 32                    # Right motor
DIR_R = 36
BRAKE_R = 38
STOP_R = 40

# Set up GPIO pins
GPIO.setup(ANALOG_L, GPIO.OUT)
GPIO.setup(DIR_L, GPIO.OUT)
GPIO.setup(BRAKE_L, GPIO.OUT)
GPIO.setup(STOP_L, GPIO.OUT)

GPIO.setup(ANALOG_R, GPIO.OUT)
GPIO.setup(DIR_R, GPIO.OUT)
GPIO.setup(BRAKE_R, GPIO.OUT)
GPIO.setup(STOP_R, GPIO.OUT)

# Create PWM instance with frequency
AnL_PWM = GPIO.PWM(ANALOG_L,1000)
AnR_PWM = GPIO.PWM(ANALOG_R,1000)

# Start PWM of required duty cycle
AnL_PWM.start(0)
AnR_PWM.start(0)

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = True	# Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0

# Some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Read the gyro and acceleromater values from MPU6050
def MPU_Init():
	# Write to sample rate register
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

	# Write to power management register
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

	# Write to Configuration register
	# Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
	bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

	# Write to Gyro configuration register
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

	# Write to interrupt enable register
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)

def read_raw_data(addr):
	# Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(DeviceAddress, addr)
        low = bus.read_byte_data(DeviceAddress, addr+1)

        # Concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68    # MPU6050 device address
MPU_Init()

time.sleep(1)

# Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)
# print(accX,accY,accZ)

if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg
print(roll)

kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll
gyroYAngle = pitch
compAngleX = roll
compAngleY = pitch

Setpoint = 180                  # Set the value when the robot is perpendicular to ground
Kp = 21                         # Set this first
Kd = 0.8                        # Set this secound
Ki = 140                        # Finally set this
pid = PID(Kp, Kd, Ki, Setpoint)

pid.auto_mode = True            # PID is enabled again
pid.sample_time = 0.01          # Update every 0.01 seconds
pid.output_limits = (0, 100)    # Output value will be between 0 and 10

timer = time.time()
flag = 0
while True:
	if(flag >100): # Problem with the connection
		print("There is a problem with the connection")
		flag=0
		continue
	try:
		# Read Accelerometer raw value
		accX = read_raw_data(ACCEL_XOUT_H)
		accY = read_raw_data(ACCEL_YOUT_H)
		accZ = read_raw_data(ACCEL_ZOUT_H)

	    # Read Gyroscope raw value
		gyroX = read_raw_data(GYRO_XOUT_H)
		gyroY = read_raw_data(GYRO_YOUT_H)
		gyroZ = read_raw_data(GYRO_ZOUT_H)

		dt = time.time() - timer
		timer = time.time()

		if (RestrictPitch):
			roll = math.atan2(accY,accZ) * radToDeg
			pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
		else:
			roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
			pitch = math.atan2(-accX,accZ) * radToDeg

		gyroXRate = gyroX/131
		gyroYRate = gyroY/131

		if (RestrictPitch):
			if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
				kalmanX.setAngle(roll)
				complAngleX = roll
				kalAngleX   = roll
				gyroXAngle  = roll
			else:
				kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

			if(abs(kalAngleX)>90):
				gyroYRate  = -gyroYRate
				kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
		else:
			if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
				kalmanY.setAngle(pitch)
				complAngleY = pitch
				kalAngleY   = pitch
				gyroYAngle  = pitch
			else:
				kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

			if(abs(kalAngleY)>90):
				gyroXRate  = -gyroXRate
				kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

		# angle = (rate of change of angle) * change in time
		gyroXAngle = gyroXRate * dt
		gyroYAngle = gyroYAngle * dt

		# compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
		compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
		compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

		if ((gyroXAngle < -180) or (gyroXAngle > 180)):
			gyroXAngle = kalAngleX
		if ((gyroYAngle < -180) or (gyroYAngle > 180)):
			gyroYAngle = kalAngleY

		print("Pitch angle: " + str(kalAngleX))

		Pitch = kalAngleX
		Control = pid(Pitch)
		# Serial.print(input); Serial.print(" =>"); Serial.println(output);

		if (input > 130) and (input < 230):      # If the robot is falling
			if (Control > 0):                    # Falling towards front
				# Drive the motor clockwise: forward
				GPIO.output(DIR_L, GPIO.HIGH)
				GPIO.output(DIR_R, GPIO.LOW)

				GPIO.output(ANALOG_L, Control)
				GPIO.output(ANALOG_R, Control)

			elif (Control < 0):                  # Falling towards back
				# Drive the motor counterclockwise: backward
				GPIO.output(DIR_L, GPIO.LOW)
				GPIO.output(DIR_R, GPIO.HIGH)

				GPIO.output(ANALOG_L, -1*Control)
				GPIO.output(ANALOG_R, -1*Control)

			else:                                 # If robot not falling
				GPIO.output(ANALOG_L, GPIO.LOW)
				GPIO.output(ANALOG_R, GPIO.LOW)

		time.sleep(0.005)

	except:
		flag += 1