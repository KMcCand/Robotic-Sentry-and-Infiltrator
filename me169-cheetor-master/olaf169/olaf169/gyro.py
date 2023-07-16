#!/usr/bin/env python3
#
#   gyro_notquite.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#
#   Create a Gyro object to read the gyroscope from the IMU.  This
#   uses the I2C bus and has to be used together with driver2.py.
#
import math
import smbus
import time
import struct 


#
#   Gyro Object
#
#   This implements the gyro readings.
#
class Gyro:
    # I2C Definitions and Communication
    I2C_ADDR = 0x68             # I2C Device Number

    REG_CONFIG   = 0x1A         # Register to set configuration
    REG_GYROCFG  = 0x1B         # Register to set gyro config
    REG_GYROX    = 0x43         # Registers to read gyro X axis
    REG_GYROY    = 0x45         # Registers to read gyro Y axis
    REG_GYROZ    = 0x47         # Registers to read gyro Z axis
    REG_PWRMGMT1 = 0x6B         # Register to set power management
    REG_ADDR     = 0x75         # Register reporting I2C address

    # Read/Write an I2C bus register.
    def readReg(self, reg):
        return self.i2cbus.read_byte_data(self.I2C_ADDR, reg)
    def writeReg(self, reg, byte):
        self.i2cbus.write_byte_data(self.I2C_ADDR, reg, byte)

    # Burst read/write multiple I2C bus registers.
    def readRegList(self, reg, N):
        return self.i2cbus.read_i2c_block_data(self.I2C_ADDR, reg, N)
    def writeRegList(self, reg, bytelist):
        self.i2cbus.write_i2c_block_data(self.I2C_ADDR, reg, bytelist)


    # Initialize.
    def __init__(self, i2cbus, scale = math.radians(500.0)):
        # Save the I2C bus object.
        self.i2cbus = i2cbus
    
        # Confirm a connection to the IMU and gyro.
        if (self.readReg(self.REG_ADDR) != self.I2C_ADDR):
            raise Exception("IMU not connected!")

        # Set the clock source to match Gyro Z (more precise).
        self.writeReg(self.REG_PWRMGMT1, 0x03)

        # Set the gyroscope low-pass filter to 42Hz so we have to
        # sample at >=42Hz to avoid aliasing.
        self.writeReg(self.REG_CONFIG, 0x03)

        # Wait 50ms to let the setup and filter change settle.
        time.sleep(0.05)

        # Set the gyro scale (default 500 deg/sec).  Feel free to change.
        self.setscale(scale)

        # Set the offset by calibration.
        self.offset = self.calibrate()

        # Assume the current reading is thus zero.
        self.reading = (0.0, False)

        # Report.
        print("Gyro enabled.")

    # Cleanup.
    def shutdown(self):
        # Nothing to do.
        pass


    # Set the Gyro scale (in rad/sec).
    def setscale(self, scale):
        # Compute the range (250, 500, 1000, or 2000 deg/sec).
        scalenum = int(math.ceil(math.log2(scale / math.radians(250.0))))
        scalenum = min(max(scalenum, 0), 3)

        # Determine and set the actual scale.
        self.scale = math.radians(250.0) * (2 ** scalenum)
        self.writeReg(self.REG_GYROCFG, scalenum << 3)

        # Let the change take effect before the next sample is read.
        time.sleep(0.01)

        # Report.
        print("Setting gyro scale to %.3f rad/sec (%.0f deg/sec)"
              % (self.scale, math.degrees(self.scale)))

    # Calibrate the Gyro Offset (assuming the IMU is not moving!).
    def calibrate(self, N = 200):
        # Report.
        print("Measuring the gyro offset - please put down/don't move")

        # Grab the samples.
        sum  = 0.0
        sum2 = 0.0
        for i in range(N):
            (speed, _) = self.readraw()
            sum  = sum  + speed
            sum2 = sum2 + speed**2
            time.sleep(0.01)
        avg = sum/N
        std = math.sqrt((sum2 - N*avg**2)/(N-1))

        # Report and check whether the std is above an acceptable
        # limit which would imply movement.
        stdlim = 0.01
        print("Gyro offset %.3f rad/sec (std %.3f <= %.3f limit)"
              % (avg, std, stdlim))
        if (std > stdlim):
            raise Exception("IMU was held or moving during gyro calibration")

        # Return the offset, being the average reading.
        return avg


    def readraw(self):
        # Grab the high (first) and low byte (second) in one read.
        bytes = self.readRegList(self.REG_GYROZ, 2)

        # Convert into a signed 16bit number.
        # FIXME: Given bytes[0] and bytes[1], create a single
        #        16bit SIGNED integer value:
        value = (bytes[0] << 8) | bytes[1]
        # How do you make sure the sign is good?
        signed_val = struct.unpack('h', struct.pack('H', value))[0]

        # Check for saturation.
        saturated = ((value > 32700) or (value < -32700))

        # Scale into rad/sec.
        # FIXME: Give the integer value (value) and the known full scale
        #        (self.scale), please determine the raw angular
        #        velocity in rad/sec (the offset is subtracted below).
        omegaraw = self.scale * signed_val / (32767) 
       
        # Return the speed and saturation flag.
        return (omegaraw, saturated)


    def read(self):
        # Place the code in a try statement, in case the read fails.
        try:
            # Take the reading.
            (omega, saturated) = self.readraw()

            # Subtract the offset and save the reading.
            self.reading = (omega - self.offset, saturated)

        except Exception as e:
            # Do not update the reading.
            print(f"Exception: {e}")
            pass

        # Return the reading (speed and saturation flag).
        return self.reading


#
#   Main
#
def main(args=None):
    # Grab the I2C bus.
    i2cbus = smbus.SMBus(1)
    
    # Initialize the motor gyro.
    # gyro = Gyro(i2cbus, math.radians(200.0))
    gyro = Gyro(i2cbus)

    # Try reading.
    try:
        original_orientation = 0
        prevTime = time.time()

        while 1:
            (omega, sat) = gyro.read()
            newTime = time.time()
            dt = newTime - prevTime
            original_orientation += dt * omega
            prevTime = newTime
            print("GyroZ = %7.3f rad/sec (sat = %d) Absolute Orientation = %7.3f " % (omega, sat, original_orientation))
            time.sleep(0.01)
    except:
        print("Breaking the loop...")

    # Cleanup (does nothing).
    gyro.shutdown()

if __name__ == "__main__":
    main()
