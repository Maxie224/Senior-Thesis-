""" Taking off using the library from GitHub """

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
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

""" disco = Plane(connection_string, None)
disco.clear_mission()
disco.set_airspeed(20) """

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.parameter['TKOFF_THR_MINACC']=9