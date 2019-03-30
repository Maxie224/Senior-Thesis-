from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from droneapi.lib import VehicleMode
from pymavlink import mavutil
# from plane import Plane

import time
import math
import numpy as np 
import psutil
import copy   
import argparse

import sys
sys.path.append("/home/apsync/dronekit-python/")
print(sys.path)

def set_servo(vehicle, servo_number, pwm_value):
	pwm_value_int = int(pwm_value)
	msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
		0,
		servo_number,
		pwm_value_int,
		0,0,0,0,0
		)
	vehicle.send_mavlink(msg)

	# mavutil.set_servo(servo_number, pwm_value)

	# master.mav.command_long_send(
	#     master.target_system, master.target_component,
	#     mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
	#     channel, pwm, 0, 0, 0, 0, 0)

# Initialize the Vehicle
connection_string = 'com6'

# Initialize the Vehicle
vehicle = connect(connection_string, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)
print 'Connection Successful!'

vehicle.parameters['ARMING_CHECK']=0
while not vehicle.parameters['ARMING_CHECK']==0:
	time.sleep(1)
print 'Arming Check Disabled'
print vehicle.parameters['SERVO9_FUNCTION']

print 'Arming motors'
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
time.sleep(1)

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
	print 'Waiting for arming...'
	time.sleep(1)
	
print 'Vehicle Armed!'

# for x in range(1, 16):
# 	set_servo(vehicle, x, 1100)
# 	time.sleep(2)
# 	set_servo(vehicle, x, 1900)
# 	time.sleep(2)
# 	set_servo(vehicle, x, 1100)
# 	print('Servo', x,'Tested')

msg = []
for x in range(1, 16):
	servo_string = 'SERVO' + str(x) + '_FUNCTION'
	vehicle.parameters[servo_string] = 0
	print 'Servo String: %i' % (vehicle.parameters[servo_string])
	temp = vehicle.message_factory.command_long_encode(
    	0, 0,    # target_system, target_component
    	mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, #command
    	0, #confirmation
    	x ,    # param 1, servo No
    	1500,          # param 2, pwm
    	1,          # param 3, repeat #
    	2, # param 4, delay
    	0, 0, 0)    # param 5 ~ 7 not used
	msg.append(temp)

# Send command to vehicle
for i in msg:
	vehicle.send_mavlink(i)