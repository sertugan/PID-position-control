#!/usr/bin/python
import RPi.GPIO as GPIO
import smbus
import math
import time
import datetime
import math
import os
#Start the pigpiod
os.system("sudo pigpiod")
time.sleep(0.1)
import pigpio
pi = pigpio.pi()

#---Kalman Accel-Gyro---#
Acceleration_angle=[0,0]
Gyro_angle=[0,0]
Total_angle=[0,0]
rad_to_deg = 180/math.pi

#//////PID CONSTANTS//////#
kp = 0.028
ki = 0.000001
kd = 0.001
#/////////////////////////#
pid_p = 0.0
pid_i = 0.0
pid_d = 0.0

desired_angle  = -2 #Desired angle set to 0 degree or adjustable angle
previous_error = 0

pwmLeftVal  = 0
pwmRightVal = 0
pwmMax      = 2000
pwmMin      = 1000
pwmLeftPin  = 4
pwmRightPin  = 23



# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
    
def get_Angle(axis,elapsedTime):
    
    gyroskop_xout = read_word_2c(0x43)
    gyroskop_yout = read_word_2c(0x45)
    gyroskop_zout = read_word_2c(0x47)
 
    acceleration_xout = read_word_2c(0x3b)
    acceleration_yout = read_word_2c(0x3d)
    acceleration_zout = read_word_2c(0x3f)
		 
    acceleration_xout_scaled = acceleration_xout / 16384.0
    acceleration_yout_scaled = acceleration_yout / 16384.0
    acceleration_zout_scaled = acceleration_zout / 16384.0
    #16384.0 that's the value that the datasheet gives us

    gyroskop_xout_scaled = gyroskop_xout / 131.0
    gyroskop_yout_scaled = gyroskop_yout / 131.0
    gyroskop_zout_scaled = gyroskop_zout / 131.0
    #131.0 that's the value that the datasheet gives us

    #---X---#
    Acceleration_angle[0] = math.atan(acceleration_yout_scaled/math.sqrt(math.pow(acceleration_xout_scaled,2)+
                                                                         math.pow(acceleration_zout_scaled,2)))*rad_to_deg
    #---Y---#
    Acceleration_angle[1] = math.atan(-1*acceleration_xout_scaled/math.sqrt(math.pow(acceleration_yout_scaled,2)+
                                                                         math.pow(acceleration_zout_scaled,2)))*rad_to_deg
    #---X---#
    Gyro_angle[0]=gyroskop_xout_scaled
    #---Y---#
    Gyro_angle[1]=gyroskop_yout_scaled
    
    #Finaly we can apply the final filter (Kalman Filter)
    #---X axis angle---#
    Total_angle[0] = 0.98*(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    #---Y axis angle---#
    Total_angle[1] = 0.98*(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

    if(axis == "X"):
        #print("X: ",Total_angle[0])
        return Total_angle[0]
    elif(axis == "Y"):
        #print("Y: ",Total_angle[1])
        return Total_angle[1]

bus = smbus.SMBus(1)
address = 0x68       
bus.write_byte_data(address, power_mgmt_1, 0)

#---Serial Communication---#
h1 = pi.serial_open("/dev/serial0",230400)

#//////Initial PWM//////#
pi.set_servo_pulsewidth(pwmLeftPin, 900)
pi.set_servo_pulsewidth(pwmRightPin,900)
time.sleep(3)
Time = datetime.datetime.now()
try:
    while True:
        timePrev    = Time
        Time        = datetime.datetime.now()
        elapsedTime = Time - timePrev
        elapsedTime = elapsedTime.seconds + elapsedTime.microseconds*0.000001#us to second

        #////// PID //////#
        angle = get_Angle("X",elapsedTime) 
        error = angle - desired_angle

        #proportional value of the PID is proportional constant multiplied by error
        pid_p = kp*error
        
        #integral value of the PID is sum the previous integral value with the error multiplied by the integral constant
        pid_i = pid_i+(ki*error)

        #derivative value of the PID is speed of change of error multiplied by derivative the constant
        pid_d = kd*((error - previous_error)/elapsedTime)

        previous_error = error #Remember to store the previous error

        #PID value is the sum of each of this 3 parts
        PID = pid_p + pid_i + pid_d
 
        #set PWM to drive BDCM (Brushless DC Motor)
        pwmLeftVal = pwmLeftVal + PID
        pwmRightVal = pwmRightVal - PID
        
        if pwmLeftVal < pwmMin:
            pwmLeftVal = pwmMin

        elif pwmLeftVal > pwmMax:
            pwmLeftVal = pwmMax

        if pwmRightVal < pwmMin:
            pwmRightVal = pwmMin

        elif pwmRightVal > pwmMax:
            pwmRightVal = pwmMax
        
        pi.set_servo_pulsewidth(pwmLeftPin,pwmLeftVal)
        pi.set_servo_pulsewidth(pwmRightPin,pwmRightVal)
        pi.serial_write(h1,(str(angle)+",").encode('utf-8'))
        pi.serial_write(h1,(str(pwmLeftVal)+",").encode('utf-8'))
        pi.serial_write(h1,(str(desired_angle)+"\n").encode('utf-8'))
        #time.sleep(0.02)
except:
    pi.stop()
    os.system("sudo pkill pigpiod")
