import csv, sys
import numpy as np
import math

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import psutil
import copy   
import argparse

# Calculate distance in Cartesian 3D-space
def getDistance(x1, y1, z1, x2, y2, z2):
	distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
	return distance

# Initialize
index_x_trans = 2
index_y_trans = 3
index_z_trans = 4
num_coords = 3
row_axis = 1

# Set print options so that all array elements are printed
np.set_printoptions(threshold=sys.maxsize)

# Extract sample drone data from CSV
mock_drone_trans = np.array([])
mock_drone_count = 0
with open('data_mock_drone.csv') as mock_drone_csv:
	csv_reader = csv.reader(mock_drone_csv, delimiter=',')
	for row in csv_reader:
		mock_drone_trans = np.append(mock_drone_trans, [float(row[index_x_trans]), float(row[index_y_trans]), float(row[index_z_trans])])
		mock_drone_count += 1
mock_drone_trans = mock_drone_trans.reshape(mock_drone_count, num_coords)
print('Processed mock drone data: %d lines and %d total results') % (mock_drone_count, mock_drone_count * num_coords)

# Extract sample target data from CSV
mock_target_trans = np.array([])
mock_target_count = 0
with open('data_mock_target.csv') as mock_target_csv:
	csv_reader = csv.reader(mock_target_csv, delimiter=',')
	for row in csv_reader:
		mock_target_trans = np.append(mock_target_trans, [float(row[index_x_trans]), float(row[index_y_trans]), float(row[index_z_trans])])
		mock_target_count += 1
mock_target_trans = mock_target_trans.reshape(mock_target_count, num_coords)
print('Processed mock target data: %d lines and %d total results') % (mock_target_count, mock_target_count * num_coords)


# ---------- DRONE CODE ----------

# connection_string = '/dev/tty.usbserial-DN02RJOC' # MAC
connection_string = 'com6'

# Initialize the Vehicle
vehicle = connect(connection_string, baud=57600, wait_ready=False)
vehicle.wait_ready(True, timeout=300)
print('Connection Successful!')

# Disabling arming check
vehicle.parameters['ARMING_CHECK']=0
while not vehicle.parameters['ARMING_CHECK']==0:
	time.sleep(1)
print("Arming Check Disabled")

# Arming the vehicle
print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
time.sleep(1)

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
	print(" Waiting for arming...")
	time.sleep(1)
print('Vehicle Armed!')


# ---------- BACK TO LOOKING AT DATA ----------

# Calculate distances for each set of coordinates
mock_distance = np.array([])
for i in range(mock_drone_count):
	x1 = 0
	y1 = 0
	z1 = 0
	x2 = 0
	y2 = 0
	z2 = 0
	for j in range(num_coords):
		if j == 0:
			x1 = mock_drone_trans[i][j]
			x2 = mock_target_trans[i][j]
		elif j == 1:
			y1 = mock_drone_trans[i][j]
			y2 = mock_target_trans[i][j]
		else:
			z1 = mock_drone_trans[i][j]
			z2 = mock_target_trans[i][j]
	distance = getDistance(x1, y1, z1, x2, y2, z2)
	print('Distance between points on line %d: %f') % (i, distance)

	# If within a certain distance, activate servo
	if (distance < 599.05):
		print('---------- TOO CLOSE ----------')

		# ---------- DRONE CODE: MOVE SERVOS ---------- 
		print('Moving servos')
		vehicle.channels.overrides['5'] = 1900
		time.sleep(2)
		vehicle.channels.overrides['5'] = 1100
		time.sleep(2)