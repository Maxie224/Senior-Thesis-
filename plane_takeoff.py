""" Taking off using the library from GitHub """

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

connection_string = '/dev/tty.usbserial-DN02RJOC'

# Initialize the Vehicle
disco = Plane(connection_string, None)

print('Connection Successful!')


# Print the Vehicle's Current State
# Get all vehicle attributes (state)
print("VEHICLE CURRENT STATE")
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % disco.vehicle.version
print "   Major version number: %s" % disco.vehicle.version.major
print "   Minor version number: %s" % disco.vehicle.version.minor
print "   Patch version number: %s" % disco.vehicle.version.patch
print "   Release type: %s" % disco.vehicle.version.release_type()
print "   Release version: %s" % disco.vehicle.version.release_version()
print "   Stable release?: %s" % disco.vehicle.version.is_stable()
print " Global Location: %s" % disco.vehicle.location.global_frame
print " Global Location (relative altitude): %s" % disco.vehicle.location.global_relative_frame
print " Local Location: %s" % disco.vehicle.location.local_frame
print " Attitude: %s" % disco.vehicle.attitude
print " Velocity: %s" % disco.vehicle.velocity
print " GPS: %s" % disco.vehicle.gps_0
print " Gimbal status: %s" % disco.vehicle.gimbal
print " Battery: %s" % disco.vehicle.battery
print " EKF OK?: %s" % disco.vehicle.ekf_ok
print " Last Heartbeat: %s" % disco.vehicle.last_heartbeat
print " Heading: %s" % disco.vehicle.heading
print " Is Armable?: %s" % disco.vehicle.is_armable
print " System status: %s" % disco.vehicle.system_status.state
print " Groundspeed: %s" % disco.vehicle.groundspeed    # settable

# Prepare Parameters for Hand Takeoff

disco.vehicle.parameters['TKOFF_THR_MINACC']=0
for x in range(1,5):
    #Callbacks may not be updated for a few seconds
    if disco.vehicle.parameters['TKOFF_THR_MINACC']==0:
        break
    time.sleep(1)
print("Minimum Takeoff Acceleration set to 0m/s/s")

disco.vehicle.parameters['TKOFF_THR_DELAY']=0
for x in range(1,5):
    #Callbacks may not be updated for a few seconds
    if disco.vehicle.parameters['TKOFF_THR_DELAY']==0:
        break
    time.sleep(1)
print("Takeoff Delay set to 0 Seconds")

# Arming and Taking off
disco.arm_and_takeoff(altitude=10, pitch_deg=12)


print("Setting Airspeed to 15")
disco.set_airspeed(15)
original_location = disco.vehicle.location.global_relative_frame


new_target = disco._get_location_metres(original_location, 10, 5)
disco.goto(new_target)
print(new_target)
	
while not disco.vehicle.location.global_relative_frame == new_target:
	print("Traveling...")
	# Statement Breaking if the Disco sees a target
	time.sleep(2)
	
original_location = disco.vehicle.location.global_relative_frame
new_target = disco._get_location_metres(original_location, -10, 5)
disco.goto(new_target)
print(new_target)
	
while not disco.vehicle.location.global_relative_frame == new_target:
	print("Traveling...")
	# Statement Breaking if the Disco sees a target
	time.sleep(2)
	
original_location = disco.vehicle.location.global_relative_frame
new_target = disco._get_location_metres(original_location, 10, 5)
disco.goto(new_target)
print(new_target)
	
while not disco.vehicle.location.global_relative_frame == new_target:
	print("Traveling...")
	# Statement Breaking if the Disco sees a target
	time.sleep(2)
	
original_location = disco.vehicle.location.global_relative_frame
new_target = disco._get_location_metres(original_location, -10, -5)
disco.goto(new_target)
print(new_target)
	
while not disco.vehicle.location.global_relative_frame == new_target:
	print("Traveling...")
	# Statement Breaking if the Disco sees a target
	time.sleep(2)
	
print('Switching to RC for Landing')
disco.set_ap_mode('MANUAL')
