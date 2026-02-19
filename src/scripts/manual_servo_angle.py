from servo2 import ServoMotor
import board
from time import sleep

# Initialize the servo on GPIO pin 14
# min_pulse_width and max_pulse_width may need to be adjusted for your servo
servo = ServoMotor(board.D18)

# Main program loop
try:
    while True:
        angle = int(input("Enter flap angle (0 to 45): "))  # User input for angle
        servo.rotate(angle)  # Set servo to entered angle
except KeyboardInterrupt:
    print("Program stopped by user")