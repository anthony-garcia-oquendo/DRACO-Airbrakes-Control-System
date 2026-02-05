import logging
import board 
from gpiozero
from time

class ServoMotor:
    MIN_ANGLE = 0
    MAX_ANGLE = 180
    MIN_PULSE_WIDTH = 0.5/1000
    MAX_PULSE_WIDTH = 2.5/1000

    def __init__(self, pin):
        self.motor = gpiozero.AngularServo(pin, 
                                            min_angle=MIN_ANGLE, 
                                            max_angle=MAX_ANGLE, 
                                            min_pulse_width=MIN_PULSE_WIDTH, 
                                            max_pulse_width=MAX_PULSE_WIDTH)
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

        logging.debug(f"Actuating servo motor to {n} degrees.")
        self.motor.angle = n
        self.last_angle = n

    def test_rotation(self, delta = 1, wait = 1):
        sum = 1
        while True:
            if sum == 45 or sum == 0:
                delta *= -1
            sum += delta
            # print(sum)
            self.rotate(sum)
            time.sleep(wait)
            
        
    def show_board_pins(self):
        pins = dir(board)
        for i in range(len(pins)):
            print(pins[i])

        
