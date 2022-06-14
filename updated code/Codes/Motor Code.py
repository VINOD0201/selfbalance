# Test motors
#!/usr/bin/env python

import time
import RPi.GPIO as GPIO

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

# Drive the motor clockwise: forward
GPIO.output(DIR_L, GPIO.HIGH)
GPIO.output(DIR_R, GPIO.LOW)

# Set the motor speed: 100/100
GPIO.output(ANALOG_L, GPIO.HIGH)
GPIO.output(ANALOG_R, GPIO.HIGH)
time.sleep(5)

# Set the motor speed: 50/100
AnL_PWM.ChangeDutyCycle(50)
AnR_PWM.ChangeDutyCycle(50)
time.sleep(5)

# Stop the motor 
AnL_PWM.ChangeDutyCycle(0)
AnR_PWM.ChangeDutyCycle(0)
time.sleep(5)

# Drive the motor counterclockwise: backward
GPIO.output(DIR_L, GPIO.LOW)
GPIO.output(DIR_R, GPIO.HIGH)

# Set the motor speed: 100/100
GPIO.output(ANALOG_L, GPIO.HIGH)
GPIO.output(ANALOG_R, GPIO.HIGH)
time.sleep(5)

# Set the motor speed: 50/100
AnL_PWM.ChangeDutyCycle(50)
AnR_PWM.ChangeDutyCycle(50)
time.sleep(5)

# Reset all the GPIO pins by setting them to LOW
GPIO.output(ANALOG_L, GPIO.LOW)
GPIO.output(DIR_L, GPIO.LOW)
GPIO.output(BRAKE_L, GPIO.LOW) 
GPIO.output(STOP_L, GPIO.LOW)

GPIO.output(ANALOG_R, GPIO.LOW)
GPIO.output(DIR_R, GPIO.LOW)
GPIO.output(BRAKE_R, GPIO.LOW) 
GPIO.output(STOP_R, GPIO.LOW)