""" Taking off using the library from GitHub """

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import math
import numpy as np 
import psutil
import copy   
import argparse

connection_string = '/dev/tty.usbserial-DN02RJOC'

# Initialize the Vehicle
vehicle = connect(connection_string, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)

print('Connection Successful!')

# Arming the Vehicle
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