#!/usr/bin/python3 

# NOTE:
# This script is intended to be run on Raspberry Pi hardware.
# It will fail on non-Blinka platforms (e.g., your laptop).

import board
import csv
from datetime import datetime
import logging
import time

from devices.sensors import BMP390, ICM20948

# Output data constants.
OUTPUT_DIRECTORY = "data/20260130_Test1"
HEADERS = [
    "Time",
    "State",
    "Flap Angle",
    "Altitude Filtered",
    "Velocity Filtered",
    "Acceleration Filtered",
    "Altitude",
    "Acceleration X",
    "Acceleration Y",
    "Acceleration Z",
    "Magnetic X",
    "Magnetic Y",
    "Magnetic Z",
    "Gyro X",
    "Gyro Y",
    "Gyro Z",
    "Temperature",
    "Pressure"
]

# Initialize various logging parameters.
now = datetime.now().strftime("%Y%m%d_%H:%M:%S")
OUTPUT_DATA_PATH = f"{OUTPUT_DIRECTORY}/data_{now}.csv"
OUTPUT_LOG_PATH = f"{OUTPUT_DIRECTORY}/log_{now}.log"
logging.basicConfig(filename=OUTPUT_LOG_PATH, level=logging.DEBUG, filemode="w")
logging.getLogger().addHandler(logging.StreamHandler()) # send log messages to terminal

# Jokes.
logging.debug("Knock knock. Who's there? Draco not Notre")
logging.debug("ACS is the all-American air brake system ... not a metric unit in sight.")
logging.debug("Disabling independent flap actuation to abide by federal missile laws.")

# Create CSV writer.
logging.debug(f"Opening output data file @ {OUTPUT_DATA_PATH}.")
writer = csv.writer(open(OUTPUT_DATA_PATH, "w+"))
writer.writerow(HEADERS)
logging.debug(f"Output data file is open @ {OUTPUT_DATA_PATH}.")

# Make the data filter.
logging.debug("Initializing the data filter.")
data_filter = DataFilter()
for _ in range(100):
    logging.debug("Processing data batch")
    data_filter.process_batch()

# # Create the devices.
# logging.debug("Creating the device drivers.")
# i2c = board.I2C()
# altimeter = BMP390(i2c)
# imu = ICM20948(i2c)
# servo = ServoMotor(board.D12)
# logging.debug("The device drivers are up and running.")

# Zero the altimeter.

