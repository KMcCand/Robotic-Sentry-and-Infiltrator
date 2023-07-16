#!/usr/bin/env python3
#
#   driver.py
#
#   Create a Driver object to process the motor driver: Prepare and
#   send commands to the motor.
#
import qwiic_scmd
import sys
import time
import traceback
import os

from itertools import chain


if os.uname()[1] == 'olafrobot2':

    # Define the constants.
    MTR_NUM_LEFT  = 0               # Motor numbers 0, 1.
    MTR_NUM_RIGHT = 1
    MTR_DIR_LEFT  = 1               # Motor forward direction (0 or 1).
    MTR_DIR_RIGHT = 0

else:
    # Define the constants.
    MTR_NUM_LEFT  = 0               # Motor numbers 0, 1.
    MTR_NUM_RIGHT = 1
    MTR_DIR_LEFT  = 1               # Motor forward direction (0 or 1).
    MTR_DIR_RIGHT = 0
    


#
#   Driver Object
#
#   This implements the left and right motor driver.  The motor
#   numbers (zero or one) are on the driver board (not GPIO pins).
#   NOTE the numbers and directions may be different for each bot!
#
class Driver:
    # Initialize.
    def __init__(self,
                 motornumL  = MTR_NUM_LEFT,  directionL = MTR_DIR_LEFT, 
                 motornumR  = MTR_NUM_RIGHT, directionR = MTR_DIR_RIGHT):
        # Pick/save the parameters.
        self.mtrL = motornumL
        self.dirL = directionL
        self.mtrR = motornumR
        self.dirR = directionR
    
        # Initialize a connection to the motor driver.
        self.driver = qwiic_scmd.QwiicScmd()
        if not self.driver.connected:
            raise Exception("Motor Driver not connected!")
        print("Motor driver connected.")

        # Begin and enable the motors (after setting commands to zero).
        self.driver.begin()
        self.driver.set_drive(0, 0, 0)
        self.driver.set_drive(1, 0, 0)
        self.driver.enable()
        print("Motors enabled.")

    # Cleanup.
    def shutdown(self):
        # Simply disable the motors.
        print("Disabling the motors...")
        self.driver.set_drive(0, 0, 0)
        self.driver.set_drive(1, 0, 0)
        self.driver.disable()

    # Set the motor pwm levels:
    def pwm(self, pwmL, pwmR):
        # Avoid the internal mapping bug -> limit -254 to 253!
        pwmL = min(max(int(pwmL), -254), 253)
        pwmR = min(max(int(pwmR), -254), 253)
        self.driver.set_drive(self.mtrL, self.dirL, pwmL)
        self.driver.set_drive(self.mtrR, self.dirR, pwmR)


#
#   Main
#
def main(args=None):
    # Initialize the motor driver.
    driver = Driver()

    # Run while watching for exceptions.
    try:
        # No argumente means ramping.
        if len(sys.argv) == 1:
            print("Ramping positive/negative...")
            # for pwm in chain(range(0,250,5),range(250,-250,-5),range(-250,0,5)):
            #     print("PWM on both motors = %3d" % pwm)
            #     driver.pwm(pwm, pwm)
            #     time.sleep(.02)
            for pwm in [200]: #[-200, -120, -60, 60, 120, 200]:
                print("PWM on both motors = %3d" % pwm)
                driver.pwm(pwm, -pwm)
                time.sleep(15)

        # Two arguments means constant levels.
        elif len(sys.argv) == 3:
            (pwmL, pwmR) = (int(sys.argv[1]), int(sys.argv[2]))
            print("Constant PWM: left %d, right %d" % (pwmL, pwmR))
            while True:
                driver.pwm(pwmL, pwmR)
                time.sleep(.05)

        # Should no occur.
        else:
            print("Usage:")
            print("  No arguments: Ramp both motors positive/negative")
            print(" Two arguments: Constant left/right pwm levels")
            raise Exception("Illegal arguments")

    # Catch exceptions, so we can cleanly disable the motors.
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    
    # Cleanup (disabling the motors).
    driver.shutdown()

if __name__ == "__main__":
    main()
