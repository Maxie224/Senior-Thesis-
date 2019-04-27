#!/usr/bin/env python2
#
# Project Eagle Eye
# Group 15 - UniSA 2015
# Gwilyn Saunders
# version 0.11.26
#
# Retrieves Vicon data via PyVicon
# Includes syncronized timestamp data via a R232 COM port.
#
# usage: python2 vicon_capture.py {--output <folder> | --time <in minutes> | --config <file> | --training <file>}
#

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

from eagleeye import Sleeper, EasyConfig, EasyArgs
from datetime import datetime
from serial import Serial
import csv, sys, os, math
import time as time_pkg
from python_vicon import PyVicon

# Get magnitude of distance between 2 spatial coordinates
def getDistance(x1, y1, z1, x2, y2, z2):
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    return distance

def get2dDistance(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

def main(sysargs):
    # ---------- DRONE CODE ----------
    connection_string = '/dev/tty.usbserial-DN02RJOC' # MAC
    connection_string = 'com6'

    # Initialize the Vehicle
    vehicle = connect(connection_string, baud=57600, wait_ready=False)
    vehicle.wait_ready(True, timeout=300)
    print "Connection Successful!"

    # Disabling arming check
    vehicle.parameters['ARMING_CHECK']=0
    while not vehicle.parameters['ARMING_CHECK']==0:
        time_pkg.sleep(1)
    print "Arming Check Disabled"

    # Arming the vehicle
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("MANUAL")
    vehicle.armed = True
    time_pkg.sleep(1)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time_pkg.sleep(1)
    print "Vehicle Armed!"


    # ---------- VICON CODE ----------
    # set arguments
    args = EasyArgs(sysargs)
    cfg = EasyConfig(args.config, group="capture")
    time = args.time or cfg.default_time
    output_folder = args.output or cfg.default_output
    outpath = os.path.join(output_folder, datetime.now().strftime(cfg.date_format))
    
    num_frames = int(time * cfg.framerate) + (cfg.flash_delay * 2)
    flash_at = [cfg.flash_delay, num_frames - cfg.flash_delay]
    sleeper = Sleeper(1.0 / cfg.framerate)
    
    # data directory sanity check
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    # start the serial listener
    if cfg.run_serial:
        try:
            serial = Serial(cfg.serial_device, 9600)
            serial.setRTS(0) # set at zero
        except OSError:
            print "Couldn't open serial device", cfg.serial_device
            return 1
    else:
        print "Not listening to serial"
    
    # open Vicon connection
    print "Connecting to Vicon..."
    client = PyVicon()
    client.connect(cfg.ip_address, cfg.port)
    
    if not client.isConnected():
        print "Failed to connect to Vicon! {}:{}".format(
                                            cfg.ip_address, cfg.port)
        return 1
    
    csvfiles = []
    csvwriters = {}
    
    # determine training or capture mode
    if args.training:
        # test target existance
        client.frame()
        if not cfg.trainer_target in client.subjects():
            print "Cannot find:", cfg.trainer_target
            return 1
        
        f = open(args.training, 'wb')
        csvfiles.append(f)
        csvwriters[cfg.trainer_target] = csv.writer(f, delimiter=cfg.output_delimiter, quoting=csv.QUOTE_MINIMAL)
        subjects = [cfg.trainer_target]
    else:
        client.frame()
        subjects = client.subjects()
        
        # open CSV files
        for sub in subjects:
            path = "{0}_{1}.csv".format(outpath, sub)
            f = open(path, 'wb')
            w = csv.writer(f, delimiter=cfg.output_delimiter, quoting=csv.QUOTE_MINIMAL)
            csvfiles.append(f)
            csvwriters[sub] = w
        
    
    # print status
    print ""
    print "Using config:", cfg._path
    print "Running for", time, "seconds ({} frames)".format(num_frames)
    print "Flash delay at:", cfg.flash_delay, " ({} seconds)".format(int(cfg.flash_delay / cfg.framerate))
    print "Capturing at", cfg.framerate, "frames per second"
    
    if args.training:
        print "Recording training target:", cfg.trainer_target, "into:", args.training
    else:
        print "Saving data into:", output_folder
        print "Recording these subjects:\n", ", ".join(subjects)
    
    print ""
    
    # Initialization before main loop
    is_deployed = False
    x_index = 0
    y_index = 1
    z_index = 2

    # # Calculate distance between two sample objects to calculate conversion between Vicon distance units and metric units
    count = 0
    sample_x1 = None
    sample_y1 = None
    sample_z1 = None
    sample_x2 = None
    sample_y2 = None
    sample_z2 = None
    control_distance = 2. * 0.3048 # ft --> m
    vwc_conversion = None
    for s in subjects:
        if count == 0:
            sample_x1 = client.translation(s)[x_index] # VWC
            sample_y1 = client.translation(s)[y_index] # VWC
            sample_z1 = client.translation(s)[z_index] # VWC
            print "Coordinates of object 1: (%f, %f, %f)" % (sample_x1, sample_y1, sample_z1)
            count += 1
        elif count == 1:
            sample_x2 = client.translation(s)[x_index] # VWC
            sample_y2 = client.translation(s)[y_index] # VWC
            sample_z2 = client.translation(s)[z_index] # VWC
            print "Coordinates of object 2: (%f, %f, %f)" % (sample_x2, sample_y2, sample_z2)

            sample_distance = getDistance(sample_x1, sample_y1, sample_z1, sample_x2, sample_y2, sample_z2) # VWC
            print "Distance between objects: %f VWC" % (sample_distance)
            vwc_conversion = sample_distance / control_distance # VWC / m
            print "Meter to VWC conversion factor: %f" % (vwc_conversion) # Hard code: About 1039.55 VWC / m
    vwc_conversion = 1039.55 # VWC / m
    g = 9.81 * vwc_conversion # VWC / s^2
    print "Acceleration due to gravity g = %f VWC / s^2" % (g)

    # Save initial position and time
    drone_obj = 'drone'
    target_obj = 'mock_target_1'
    drone_x1 = client.translation(drone_obj)[x_index] # VWC
    drone_y1 = client.translation(drone_obj)[y_index] # VWC
    drone_z1 = client.translation(drone_obj)[z_index] # VWC
    drone_x2 = None
    drone_y2 = None
    drone_z2 = None
    t0 = time_pkg.time() # s
    print "Initial position: (%f, %f, %f)" % (drone_x1, drone_y1, drone_z1)
    print "Initial time: %s s" % (t0)

    # Initialize
    distance_threshold = float('-inf')
    distance = None
    drone_vx = None
    drone_vy = None
    drone_vz = None

    # Main loop
    print "---------- MAIN LOOP START ----------\n"
    for c in range(0, num_frames):
        sleeper.stamp()
        
        # run flash
        flash = "."
        if cfg.run_serial:
            if c in flash_at:
                serial.setRTS(1)
                flash = "F"
                sys.stdout.write("\r - - - - - - - Flash!\r")
            else: 
                serial.setRTS(0)
        
        client.frame()

        for s in subjects:
            csvwriters[s].writerow(
                [sleeper.getStamp(), flash] + 
                list(client.translation(s)) + 
                list(client.rotation(s)) + 
                list(client.markerStatus(s)) +
                [is_deployed] +
                [drone_vx, drone_vy, drone_vz] +
                [distance_threshold, distance])
        
        # Calculate instantaneous velocity of drone
        drone_x2 = client.translation(drone_obj)[x_index] # VWC
        drone_y2 = client.translation(drone_obj)[y_index] # VWC
        drone_z2 = client.translation(drone_obj)[z_index] # VWC
        t = time_pkg.time() # s
        time_passed = t - t0 # s
        x_distance = drone_x2 - drone_x1 # VWC
        y_distance = drone_y2 - drone_y1 # VWC
        z_distance = drone_z2 - drone_z1 # VWC
        drone_vx = x_distance / time_passed # VWC / s
        drone_vy = y_distance / time_passed # VWC / s
        drone_vz = z_distance / time_passed # VWC / s
        # print "X-Velocity: %f VWC / s" % (drone_vx)
        # print "Z-Velocity: %f VWC / s" % (drone_vz)

        # Calculate distance threshold (the distance the drone must be from the target)
        target_x = client.translation(target_obj)[x_index] # VWC
        target_y = client.translation(target_obj)[y_index] # VWC
        target_z = client.translation(target_obj)[z_index] # VWC
        temp_distance = math.sqrt(x_distance**2 + y_distance**2) # VWC
        time_buffer = .1 # s
        distance_threshold = math.sqrt(drone_vx**2 + drone_vy**2) * ((drone_vz + math.sqrt(drone_vz**2 + 2. * g * abs(drone_z2 - target_z))) / g + time_buffer) # VWC

        # Calculate updated distance between drone and target
        distance = get2dDistance(drone_x2, drone_y2, target_x, target_y)

        # Check if servo code should be activated
        if abs(distance) < distance_threshold:
            # ---------- SERVO CODE ----------
            print "---------- Near! Activating deployment mechanism... ----------"
            deployment_servo_ch = '6'
            vehicle.channels.overrides[deployment_servo_ch] = 1900
            time_pkg.sleep(2)
            vehicle.channels.overrides[deployment_servo_ch] = 1100
            time_pkg.sleep(2)
            print "---------- Deployed! ----------"
            print "Distance threshold: %f VWC" % (distance_threshold)
            print "2D distance between %s and %s: %f VWC" % (drone_obj, target_obj, distance)
            print "X-Velocity: %f VWC / s" % (drone_vx)
            print "Y-Velocity: %f VWC / s" % (drone_vy)
            print "Z-Velocity: %f VWC / s\n" % (drone_vz)

            is_deployed = True

        # sleep until next timestamp
        sys.stdout.write("{}/{}\r".format(c, num_frames))
        sleeper.sleep("\r - - - - - - - - - Late!\r")
        sys.stdout.flush()

        # Update initial coordinates of drone
        drone_x1 = drone_x2
        drone_y1 = drone_y2
        drone_z1 = drone_z2
        t0 = t
        time_pkg.sleep(0.1)
        
    # clean up
    for f in csvfiles: f.close()
    client.disconnect()
    
    print "\nComplete."
    return 0

if __name__ == '__main__':
    exit(main(sys.argv))
