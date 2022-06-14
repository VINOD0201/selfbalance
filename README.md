# selfbalance
Step 1: Software installation
I’m assuming that you already installed the Raspbian and you know how to use SSH to login to your Raspberry Pi. We will start from installing i2C kernel module and wiringPi library. The i2C kernel module is to help us to access MPU6050 and the WiringPi library is to help us to access GPIO.
Here are the instructions:
To install i2c kernel module,
$ sudo apt-get install libi2c-dev
To setup i2c kernel module,
$ sudo vi /etc/modules
Add following lines into the file.
i2c-bcm2708  <br>i2c-dev
We also have to check a blacklist file.
$ sudo vi /etc/modprobe.d/raspi-blacklist.conf
Make sure the following two lines are commented, then save.
#blacklist spi-bcm2708  <br>#blacklist i2c-bcm2708
Check raspi-config
$ sudo raspi-config  <br>In Advanced Options -> I2C, please enabled it.
Then, reboot your RPi
$ sudo bash; sync;sync;reboot
When it goes back, check if the i2c driver kernel module is loaded automatically.
$ lsmod |grep i2c
i2c_dev                 6027  0   <br>i2c_bcm2708             4990  0
The wiringPi is PRE-INSTALLED in Raspbian system now. We don't need to manually install it anymore.
Then, let's install the wiringPi library.
Install git first.
$ sudo apt-get install git-core
Download and install wiringPi
$ cd  <br>$ git clone git://git.drogon.net/wiringPi  <br>$ cd wiringPi  <br>$ sudo ./build

Step 2: Testing MPU6050 sensor
Now, we will make sure the MPU6050 actually works.
Let's install the i2c testing tool.
$ sudo apt-get install i2c-tools
Execute it to check if it saw the MPU6050 on i2c bus.
$ sudo i2cdetect -y 0 (for RPi Revision 1 board)    <br>  or <br>$ sudo i2cdetect -y 1 (for RPi Revision 2, Pi B, Pi B+, or Pi 2 boards)
If you saw the output as below, it saw the MPU6050. It means the mpu6050 device is on i2c bus and its i2c address is “0x68”
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f <br>00:          -- -- -- -- -- -- -- -- -- -- -- -- -- <br>10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- <br>20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- <br>30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- <br>40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- <br>50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- <br>60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- <br>70: -- -- -- -- -- -- -- --
Let's try some simple tools to get those acceleration and gyroscope information.
Download and compile following tool
$ cd <br>$ git clone <a href="https://github.com/wennycooper/mpu6050_in_c.git" rel="nofollow"> https://github.com/wennycooper/mpu6050_in_c.git <br></a>$ cd mpu6050_in_c $ cat README.md<br>$ gcc -o mpu6050_accl ./mpu6050_accl.c  -lwiringPi -lpthread -lm $ gcc -o mpu6050_gyro ./mpu6050_gyro.c  -lwiringPi -lpthread -lm
To get acceleration information, please execute
$ sudo ./mpu6050_accl
You should see something like:
My acclX_scaled: 0.140625
My acclY_scaled: -0.031006
My acclZ_scaled: 0.994141
My X rotation: -1.768799
My Y rotation: -8.047429
The above acclX, acclY & acclZ are the measured acceleration in g=9.8 m/s^2 in X, Y and Z axis. With some basic trigonometric functions calculation, we can obtain the orientation angles. They are “My X & Y rotation” and the unit is in degree.
Let's also check gyroscope measurement. Please execute:
$ sudo ./mpu6050_gyro
It will tell you the velocity (in degree/s) of angle in each axis, like this
My gyroX_scaled: -4.312977 <br>My gyroY_scaled: 0.458015 <br>My gyroZ_scaled: 0.366412 <br>My gyroX_scaled: -4.053435 <br>My gyroY_scaled: 0.427481 <br>My gyroZ_scaled: -0.160305
Please try rotate the car frame and watch the changes by time.
If we only use acceleration information to calculate the orientation angle, the angle will be not stable and not reliable. So, should use the velocity of angle information to calculate the angle, and complement with the angle from acceleration. Then it will give us more stable and reliable angle.

Step 3: Testing DC motors
Please run “Motor Code.py” by visual studio or other program to make the motors rotating.
If everything works well, first the two motors should rotate in the same direction together and then in reverse direction together. They should rotate for 5 seconds in forward direction at high speed, then at lower speed for 5 seconds, then stop, then for 5 seconds in backward direction at high speed, then at lower speed for 5 seconds and then stopped.

Step 4: Put all together
Please run code “Main Code.py”. This program measured the orientation information and drive the motors. If the angle of orientation is too large (>50 degree) to be balanced, it will stop the motors for safety.
You can see the PID control parameters in line 124 to 126 of the code. It is a useful method in automatic control theory.

Step 5: Go
Because of the torque and rotation speed limitation, the control program will fail to keep balancing if the error angle is too large. So, before running the program, please use your hand to keep the robot balanced as possible as you can.
Due to each robot has different center of mass. It causes the initial balanced angle varies. That's why I have a "Setpoint" variable in line 5 of the code. Please modify it to fit your robot.

If the robot failed to keep balancing, the problem may be:
•	The initial Setpoint is not good enough, please modify it and try more.
•	The direction of driving motor and the direction of sensor measurement are the same.
•	When start it, the initial angle is too large to be balanced. You may try it more.
•	If it is balanced but not graceful, you may try to modify PID parameters.
