import logging
import board 
import time
import os
import csv
import numpy as np
from adafruit_pca9685 import PCA9685
import busio
from adafruit_servokit import ServoKit

class ServoMotor:

    # i2c = busio.I2C(board.SCL, board.SDA)
    # pca = PCA9685(i2c)
    # pca.frequency = 60  
    # servo = pca.channels[0]

    kit = ServoKit(channels=16)
    
    # MIN_ANGLE = 0
    # MAX_ANGLE = 180
    # MIN_PULSE_WIDTH = 0.5/1000
    # MAX_PULSE_WIDTH = 2.5/1000
    FLAP_MIN = 0.0
    FLAP_MAX = 45.0
    SAFE_SERVO_ANGLE = 0.0 # servo angle that closes flaps
    


    def __init__(self, pin, flap2servo_path=None):
        self.flap_arr = None
        self.servo_arr = None
        self.last_flap_angle = 0

        if flap2servo_path is None:
            flap2servo_path = os.path.join(os.getcwd(), "docs", "lookup_tables", "flap2servo.csv")
        self.flap2servo_path = flap2servo_path
        
        # self.motor = gpiozero.AngularServo(
        #     pin, 
        #     min_angle=self.MIN_ANGLE, 
        #     max_angle=self.MAX_ANGLE, 
        #     min_pulse_width=self.MIN_PULSE_WIDTH, 
        #     max_pulse_width=self.MAX_PULSE_WIDTH
        # )

        # load calibration table (critical)
        self.map_points = self.load_flap2servo(self.flap2servo_path) 
        


    def rotate(self, n):
       
        try:
            n = float(n)
        except ValueError:
            logging.warning(f"SERVO ERROR: must pass a number, not {n} of type {type(n)}.")
            return
        
        if n < self.FLAP_MIN or n > self.FLAP_MAX:
            logging.warning(f"SERVO ERROR: unsafe actuation angle ({n}), stay in [{self.FLAP_MIN}, {self.FLAP_MAX}].")
            return

        n_servo = self.interp_servo_angle(n)

        logging.debug(f"Actuating: flap {n} degrees -> servo {n_servo:.2f} degrees.")
        # pulse_min = 500
        # pulse_max = 2500

        # pulse = pulse_min + (n_servo/180) * (pulse_max - pulse_min)
        # duty = int((pulse / 20000) * 65535)  # Convert pulse width to duty cycle (20ms period)
        # self.servo.duty_cycle = duty
        ServoMotor.kit.servo[0].angle =  n_servo
        self.last_flap_angle = n

    def test_rotation(self, delta = 1, wait = 1):
        sum = 1
        while True:
            if sum >= self.FLAP_MAX or sum <= self.FLAP_MIN:
                delta *= -1
            sum += delta
            # print(sum)
            self.rotate(sum)
            time.sleep(wait)
            
        
    def show_board_pins(self):
        for name in dir(board):
            print(name)

    def load_flap2servo(self, path):
        try:
            data = np.genfromtxt(path, delimiter=',', names=True) # names=True handles the header
            self.flap_arr = np.array(data['flap_angle'], dtype=float)
            self.servo_arr = np.array(data['servo_angle'], dtype=float)
        except FileNotFoundError:
            logging.exception(f"Servo calibration file not found: {path}")
            self.motor.angle = self.SAFE_SERVO_ANGLE
            exit(1)
        except Exception as e:
            logging.exception(f"Failed reading servo calibration CSV: {e}")
            self.motor.angle = self.SAFE_SERVO_ANGLE 
            exit(1)
        return data

    def interp_servo_angle(self, flap_angle):
        if self.flap_arr is None or self.servo_arr is None:
            logging.exception("Interpolation failed: Calibration data not loaded.")
            self.motor.angle = self.SAFE_SERVO_ANGLE
            exit(1) 
        return float(np.interp(flap_angle, self.flap_arr, self.servo_arr))    
