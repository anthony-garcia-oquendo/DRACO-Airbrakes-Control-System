from enum import Enum
import math

#physical constants in imperial units
G = 32.17405  # ft/s^2
LAUNCH_ALTITUDE = 0  # ft
LAUNCH_ACCELERATION = 0  # ft/s^2
LAUNCH_ALTITUDE_FAILSAFE = 0  # ft
BURNOUT_ALTITUDE = 0  # ft
BURNOUT_ACCELERATION = 0  # ft/s^2
BURNOUT_ALTITUDE_FAILSAFE = 0  # ft
PREDICTED_APOGEE = 0  # ft
APOGEE_VELOCITY = 0  # ft/s




class State(Enum):
    "Rocket flight states"
    PRELAUNCH = 0
    LAUNCHED = 1
    BURNOUT = 2
    OVERSHOOT = 3
    APOGEE = 4

def state_obtainer(state, altitude, velocity, acceleration):
    if state == State.PRELAUNCH:
        if (altitude > LAUNCH_ALTITUDE and acceleration > LAUNCH_ACCELERATION) or altitude > LAUNCH_ALTITUDE_FAILSAFE:
            return State.LAUNCHED
        else:
            return state
    elif state == State.LAUNCHED:
        if (altitude > BURNOUT_ALTITUDE and acceleration < BURNOUT_ACCELERATION) or altitude > BURNOUT_ALTITUDE_FAILSAFE:
            return State.BURNOUT
        else:
            return state
    elif state == State.BURNOUT:
        if altitude >= PREDICTED_APOGEE:
            return State.OVERSHOOT
        elif velocity <= APOGEE_VELOCITY:
            return state.APOGEE
        else:
            return state
    elif state == State.OVERSHOOT:
        if velocity <= APOGEE_VELOCITY:
            return State.APOGEE
        else:
            return state   
    else:
        return state
class PI_Controller:
    def __init__(self):
       self.previous_error = 0
       self.previous_time = 0
       self.previous_integral = 0
       self.previous_control =0;

