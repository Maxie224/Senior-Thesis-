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

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Initialize the Vehicle
disco = Plane(connection_string, None)

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
print " Autopilot capabilities"
print "   Supports MISSION_FLOAT message type: %s" % disco.vehicle.capabilities.mission_float
print "   Supports PARAM_FLOAT message type: %s" % disco.vehicle.capabilities.param_float
print "   Supports MISSION_INT message type: %s" % disco.vehicle.capabilities.mission_int
print "   Supports COMMAND_INT message type: %s" % disco.vehicle.capabilities.command_int
print "   Supports PARAM_UNION message type: %s" % disco.vehicle.capabilities.param_union
print "   Supports ftp for file transfers: %s" % disco.vehicle.capabilities.ftp
print "   Supports commanding attitude offboard: %s" % disco.vehicle.capabilities.set_attitude_target
print "   Supports commanding position and velocity targets in local NED frame: %s" % disco.vehicle.capabilities.set_attitude_target_local_ned
print "   Supports set position + velocity targets in global scaled integers: %s" % disco.vehicle.capabilities.set_altitude_target_global_int
print "   Supports terrain protocol / data handling: %s" % disco.vehicle.capabilities.terrain
print "   Supports direct actuator control: %s" % disco.vehicle.capabilities.set_actuator_target
print "   Supports the flight termination command: %s" % disco.vehicle.capabilities.flight_termination
print "   Supports mission_float message type: %s" % disco.vehicle.capabilities.mission_float
print "   Supports onboard compass calibration: %s" % disco.vehicle.capabilities.compass_calibration
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
print " Rangefinder: %s" % disco.vehicle.rangefinder
print " Rangefinder distance: %s" % disco.vehicle.rangefinder.distance
print " Rangefinder voltage: %s" % disco.vehicle.rangefinder.voltage
print " Heading: %s" % disco.vehicle.heading
print " Is Armable?: %s" % disco.vehicle.is_armable
print " System status: %s" % disco.vehicle.system_status.state
print " Groundspeed: %s" % disco.vehicle.groundspeed    # settable
print " Airspeed: %s" % disco.vehicle.airspeed    # settable
print " Mode: %s" % disco.vehicle.mode.name    # settable
print " Armed: %s" % disco.vehicle.armed    # settable


# Prepare Parameters for Hand Takeoff
disco.vehicle.parameters['TKOFF_THR_MINACC']=9
print("Minimum Takeoff Acceleration set to 9m/s/s")
disco.vehicle.parameters['TKOFF_THR_DELAY']=2
print("Takeoff Delay set to 0.2 Seconds")
disco.vehicle.parameters['TKOFF_THR_MINSPD']=4
print("Takeoff Minspeed set to 4m/s")
disco.vehicle.parameters['TECS_PITCH_MAX']=20
print("Maximum Pitch set to 20 Degrees")

# Arm and Set to AUTO
disco.clear_mission()
disco.arm()
disco.set_ap_mode("AUTO")
print("Mode Set, Hand Takeoff Now!")
time.sleep(10)
print("Ready to be Commanded!")

# Fly to One Location and Fly Back