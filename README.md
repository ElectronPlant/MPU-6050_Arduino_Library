# MPU-6050 Gyroscope + Accelerometer sensor library


MPU Library
---------------
 The library aims to simplify the use of the MPU-6050 sensor by including all the basic functionality such us:
 
 +) Initialization.
	 
 +) Self-test.
	 
 +) Calibration.
	 
 +) Retrieving the data from the sensor.
	 
 +) Simplified Kalman Filer to calculate the angle.
	 
	 
(It uses millis() for the timeout --> check if the timers are modified)
 

Acknowledgments:
-------------------
This file uses the I2Cdev library made by "jrowberg" which is available on Githib: https://github.com/jrowberg/i2cdevlib or on  https://www.i2cdevlib.com

The library was inspired by the mpu_6050_library also implemented by "jrowberg" which is available on Github: 
https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 
The self-test procedure was based on the one proposed by "kriswiner" which is also available on Github: 
https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino

Repository contents:
--------------------

 +) Source file: "mpu_6050_library.cpp".
 
 +) Header file: "mpu_6050_library.has".
 
 +) Test file: "mpu_test.ino".

