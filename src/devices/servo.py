import logging
import board 
import pwmio
import time

class ServoMotor:
    ON = 2 ** 16
    MOTOR_MIN = 0.5
    MOTOR_MAX = 2.5

    def __init__(self, pin, **kwargs):
        self.motor = pwmio.PWMOut(pin, variable_frequency=False, **kwargs)
        self.motor.frequency = 50
        self.last_angle = 0

    def rotate(self, n):
       
        try:
            n = float(n)
        except ValueError:
            logging.warning(f"SERVO ERROR: must pass a number, not {n} of type {type(n)}.")
            return
        
        if n < 0 or n > 45:
            logging.warning(f"SERVO ERROR: unsafe actuation angle ({n}), stay in [0, 45].")
            return

        logging.debug(f"Actuating servo motor to {n}% = {n} degrees.")
        duty = (n + 45)/1800
        self.motor.duty_cycle = int(duty * ServoMotor.ON)
        self.last_angle = n
        print(duty, ' ', self.motor.duty_cycle)

    def test_rotation(self, delta = 1, wait = 1):
        sum = 1
        while True:
            if sum == 45 or sum == 0:
                delta *= -1
            sum += delta
            # print(sum)
            self.rotate(sum)
            time.sleep(wait)
            
        
    def show_pins(self):
        pins = dir(board)
        for i in range(len(pins)):
            print(pins[i])

        
