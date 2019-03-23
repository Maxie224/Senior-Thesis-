from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from droneapi.lib import VehicleMode
from pymavlink import mavutil
from plane import Plane

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
		mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
		0,
		servo_number,
		pwm_value_int,
		0,0,0,0,0
		)
	vehicle.send_mavlink(msg)

# Initialize the Vehicle
connection_string = '/dev/tty.usbserial-DN02RJOC'

# Initialize the Vehicle
vehicle = connect(connection_string, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)
print('Connection Successful!')

vehicle.parameters['ARMING_CHECK']=0
while not vehicle.parameters['ARMING_CHECK']==0:
	time.sleep(1)
print("Arming Check Disabled")

print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True
time.sleep(1)

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
	print(" Waiting for arming...")
	time.sleep(1)
	
print('Vehicle Armed!')

for x in range(1, 60):
	set_servo(vehicle, 1, 1100)
	time.sleep(1)
	set_servo(vehicle, 1, 1500)
	time.sleep(1)
	set_servo(vehicle, 1, 1100)
	print('Servo', x ,'Tested')