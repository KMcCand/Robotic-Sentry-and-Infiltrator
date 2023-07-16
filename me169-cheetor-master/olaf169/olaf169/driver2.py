#!/usr/bin/env python3
#
#   driver2.py
#
#   Create a Driver object to process the motor driver: Prepare and
#   send commands to the motor.
#
#   Note the V2 uses the I2C bus directly, versus the Sparkfun Qwicc library!
#
import smbus
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
    MTR_DIR_RIGHT = 1

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
    # I2C Definitions and Communication
    I2C_ADDR = 0x5D             # I2C Device Number

    ID = 0xA9                   # ID, unique to the driver chip

    REG_ID     = 0x01           # Register storing ID
    REG_MOTORA = 0x20           # Register to set motor A
    REG_MOTORB = 0x21           # Register to set motor B
    REG_ENABLE = 0x70           # Register to enable/disable

    REG_MOTORX = {0:REG_MOTORA, 1:REG_MOTORB}

    # Read/Write an I2C bus register.
    def readReg(self, reg):
        return self.i2cbus.read_byte_data(self.I2C_ADDR, reg)
    def writeReg(self, reg, byte):
        self.i2cbus.write_byte_data(self.I2C_ADDR, reg, byte)


    # Initialize.
    def __init__(self, i2cbus,
                 motornumL  = MTR_NUM_LEFT,  directionL = MTR_DIR_LEFT, 
                 motornumR  = MTR_NUM_RIGHT, directionR = MTR_DIR_RIGHT):
        # Save the I2C bus object.
        self.i2cbus = i2cbus

        # Pick/save the parameters.
        self.mtrL = motornumL
        self.dirL = directionL
        self.mtrR = motornumR
        self.dirR = directionR
    
        # Confirm a connection to the motor driver.
        if (self.readReg(self.REG_ID) != self.ID):
            raise Exception("Motor Driver not connected!")
        print("Motor driver connected.")

        # Enable the motors (after setting commands to zero).
        self.set_drive(0, 0, 0)
        self.set_drive(1, 0, 0)
        self.enable()
        print("Motors enabled.")

    # Cleanup.
    def shutdown(self):
        # Simply disable the motors.
        print("Disabling the motors...")
        self.set_drive(0, 0, 0)
        self.set_drive(1, 0, 0)
        self.disable()

    # Enable/Disable.
    def enable(self):
        self.writeReg(self.REG_ENABLE, 1)
    def disable(self):
        self.writeReg(self.REG_ENABLE, 0)

    # Set the motors - simply use the same API as the SparkFun library.
    def set_drive(self, channel, reverse, value):
        # Process the value: Reverse if necessary (actually inverted),
        # scale to -127..127 (rounding to zero), and offset by 128.
        if not reverse:
            value = -value
        value = 128 + min(max(int(value/2), -127), 127)

        # Send to the driver.
        self.writeReg(self.REG_MOTORX[channel], value)

    # Set the motor pwm levels:
    def pwm(self, pwmL, pwmR):
        # Avoid the internal mapping bug -> limit -254 to 253!
        pwmL = min(max(int(pwmL), -254), 253)
        pwmR = min(max(int(pwmR), -254), 253)
        self.set_drive(self.mtrL, self.dirL, pwmL)
        self.set_drive(self.mtrR, self.dirR, pwmR)


#
#   Main
#
def main(args=None):
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)
    
    # Initialize the motor driver.
    driver = Driver(i2cbus)

    # Run while watching for exceptions.
    try:
        # No argumente means ramping.
        if len(sys.argv) == 1:
            print("Ramping positive/negative...")
            for pwm in chain(range(0,250,5),range(250,-250,-5),range(-250,0,5)):
                print("PWM on both motors = %3d" % pwm)
                driver.pwm(pwm, pwm)
                time.sleep(.02)

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
