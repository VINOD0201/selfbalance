from Kalman_Filter import KalmanAngle
import smbus
import time
import math
import smbus

class MPU6050:
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(self, address, parent_bus) -> None:
        self.parent_bus=parent_bus
        self.Device_Address = address

        # write to sample rate register
        self.parent_bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)

        # Write to power management register
        self.parent_bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

        # Write to Configuration register
        self.parent_bus.write_byte_data(self.Device_Address, self.CONFIG, 0)

        # Write to Gyro configuration register
        self.parent_bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)

        # Write to interrupt enable register
        self.parent_bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def readRawData(self, address):
        # Accelero and Gyro value are 16-bit
        high = self.parent_bus.read_byte_data(self.Device_Address, address)
        low = self.parent_bus.read_byte_data(self.Device_Address, address+1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value

    def rotationX(self):
        kalmanX = KalmanAngle()
        kalmanY = KalmanAngle()

        RestrictPitch = True
        radToDeg = 57.2957786
        kalAngleX = 0
        kalAngleY = 0

        # Read Accelerometer raw value
        accX = self.readRawData(self.ACCEL_XOUT_H)
        accY = self.readRawData(self.ACCEL_YOUT_H)
        accZ = self.readRawData(self.ACCEL_ZOUT_H)

        if (RestrictPitch):
            roll = math.atan2(accY,accZ) * radToDeg
            pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
        else:
            roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
            pitch = math.atan2(-accX,accZ) * radToDeg
        # print(roll)
        kalmanX.setAngle(roll)
        kalmanY.setAngle(pitch)
        gyroXAngle = roll
        gyroYAngle = pitch
        compAngleX = roll
        compAngleY = pitch

        timer = time.time()
        flag = 0
        if(flag >100): # Problem with the connection
            print("There is a problem with the connection")
            flag=0
        try:
            # Read Accelerometer raw value
            accX = self.readRawData(self.ACCEL_XOUT_H)
            accY = self.readRawData(self.ACCEL_YOUT_H)
            accZ = self.readRawData(self.ACCEL_ZOUT_H)

            # Read Gyroscope raw value
            gyroX = self.readRawData(self.GYRO_XOUT_H)
            gyroY = self.readRawData(self.GYRO_YOUT_H)
            gyroZ = self.readRawData(self.GYRO_ZOUT_H)

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

            # print("Angle X: " + str(kalAngleX))
            time.sleep(0.005)

        except:
            flag += 1

        return kalAngleX