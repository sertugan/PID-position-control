#!/usr/bin/python
import smbus
import math
import time
import datetime

#import RPi.GPIO as GPIO

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

pid_p = 0.0
pid_i = 0.0
pid_d = 0.0

#//////PID CONSTANTS//////#
kp = 1.2
ki = 0.0050
kd = 0.0050
#/////////////////////////#

desired_angle  = 0 #Desired angle set to 0 degree or adjustable angle
previous_error = 0

pwmLeftVal  = 0
pwmRightVal = 0
pwmMax      = 1500.0
pwmMin      = 900.0



#You can add pwmRight configuration 
#But we only set one PWM for our project because we use only one motor


    
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
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def get_Angle(rotation):
    
    gyroskop_xout = read_word_2c(0x43)
    gyroskop_yout = read_word_2c(0x45)
    gyroskop_zout = read_word_2c(0x47)
 

	 
    acceleration_xout = read_word_2c(0x3b)
    acceleration_yout = read_word_2c(0x3d)
    acceleration_zout = read_word_2c(0x3f)
		 
    acceleration_xout_scaled = acceleration_xout / 16384.0
    acceleration_yout_scaled = acceleration_yout / 16384.0
    acceleration_zout_scaled = acceleration_zout / 16384.0

    if(rotation == "X"):
       return get_x_rotation(acceleration_xout_scaled, acceleration_yout_scaled, acceleration_zout_scaled)
    elif(rotation == "Y"):
       return get_y_rotation(acceleration_xout_scaled, acceleration_yout_scaled, acceleration_zout_scaled)
 
bus = smbus.SMBus(1)
address = 0x68       

bus.write_byte_data(address, power_mgmt_1, 0)


Time = datetime.datetime.now()
#//////Initial PWM//////#

while True:
    timePrev     = Time
    Time   = datetime.datetime.now()
    elapsedTime = Time - timePrev
    elapsedTime = elapsedTime.seconds + elapsedTime.microseconds*0.000001#us to second
    
    #////// PID //////#
    error = get_Angle("X") - desired_angle

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
    	 
    #pwmLeft.ChangeDutyCycle(pwmLeftVal)
    #pwmSet("R",pwmRightVal)
    #pwmSet("L",pwmLeftVal)
    print ("ROTATION: ",get_rotation("X"))
    print ("pwmLeft: " , pwmLeftVal)
    print ("pwmRight: " , pwmRightVal)
    #time.sleep(1)
