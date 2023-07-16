#!/usr/bin/env python3
#
#   encoder_skeleton.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#
#   Create an Encoder object to process the encoders: Prepare and
#   handle the GPIO interrupts and count the encoder!
#
import pigpio
import time
import traceback
import os

# Define the constants.
if os.uname()[1] == 'olafrobot2':
    ENC_CHA_RIGHT  = 23              # Left/Right encoder channels A/B
    ENC_CHB_RIGHT  = 22             # These are the GPIO channels.
    ENC_CHA_LEFT = 24 
    ENC_CHB_LEFT = 25
else:
    ENC_CHA_RIGHT  = 22              # Left/Right encoder channels A/B
    ENC_CHB_RIGHT  = 23             # These are the GPIO channels.
    ENC_CHA_LEFT = 24 
    ENC_CHB_LEFT = 25


#
#   Encoder Object
#
#   This implements the left and right wheel encoder.  NOTE the
#   channels may be different?
#
class Encoder:
    # Initialize.
    def __init__(self,
                 chLA = ENC_CHA_LEFT,  chLB = ENC_CHB_LEFT,
                 chRA = ENC_CHA_RIGHT, chRB = ENC_CHB_RIGHT):
        # Initialize the connection to the pigpio daemon.
        self.io = pigpio.pi()
        if not self.io.connected:
            raise Exception("Unable to connection to pigpio daemon!")
        print("Connected to pigpio daemon.")

        # Setup the input pins with a pull-up resistor.  The encoders
        # (Hall Effect chip) are open drain output, i.e. only pull
        # down.  Set up all channels this way:
        for channel in [chLA, chLB, chRA, chRB]:
            print("Setting up input GPIO%2d with pull-up..." % channel)
            self.io.set_mode(channel, pigpio.INPUT) 
            self.io.set_pull_up_down(channel, pigpio.PUD_UP) 

        # Prepare/initialize the channel states.
        self.LA = self.io.read(chLA)
        self.LB = self.io.read(chLB)
        self.RA = self.io.read(chRA)
        self.RB = self.io.read(chRB)

        # Prepare/initialize the encoder counts.
        self.lcount = 0
        self.rcount = 0

        # Finally, prepare the callbacks, setting things up.
        print("Starting the callback functions...")
        self.cbs = [self.io.callback(chLA, pigpio.RISING_EDGE,  self.LArise),
                    self.io.callback(chLB, pigpio.RISING_EDGE,  self.LBrise),
                    self.io.callback(chRA, pigpio.RISING_EDGE,  self.RArise),
                    self.io.callback(chRB, pigpio.RISING_EDGE,  self.RBrise),
                    self.io.callback(chLA, pigpio.FALLING_EDGE, self.LAfall),
                    self.io.callback(chLB, pigpio.FALLING_EDGE, self.LBfall),
                    self.io.callback(chRA, pigpio.FALLING_EDGE, self.RAfall),
                    self.io.callback(chRB, pigpio.FALLING_EDGE, self.RBfall)]

        # Report and return.
        print("Encoders reading.")
      

    # Cleanup.
    def shutdown(self):
        # Simply cancel the callback functions, waiting until all is done.
        print("Cancelling the encoder callback functions...")
        for cb in self.cbs:
            cb.cancel()
        time.sleep(0.25)

        # Finally stop the I/O.
        print("Stopping the GPIO connection...")
        self.io.stop()

    # Report
    def left(self):
        return (self.lcount)
    def right(self):
        return (self.rcount)

    # Callback Functions:
    def LArise(self, gpio, level, tick):
        # Update the count
        if (not self.LB):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state.
        self.LA = True

    def LAfall(self, gpio, level, tick):
        # Update the count
        if (self.LB):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state.
        self.LA = False

    def LBfall(self, gpio, level, tick):
        # Update the count
        if (not self.LA):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state.
        self.LB = False

    def LBrise(self, gpio, level, tick):
        # Update the count
        if (self.LA):
            self.lcount += 1
        else:
            self.lcount -= 1
        # Save the new state.
        self.LB = True

    def RArise(self, gpio, level, tick):
        # Update the count
        if (not self.RB):
            self.rcount += 1
        else:
            self.rcount -= 1
        # Save the new state.
        self.RA = True

    def RAfall(self, gpio, level, tick):
        # Update the count
        if (self.RB):
            self.rcount += 1
        else:
            self.rcount -= 1
        # Save the new state.
        self.RA = False

    def RBfall(self, gpio, level, tick):
        # Update the count
        if (not self.RA):
            self.rcount += 1
        else:
            self.rcount -= 1
        # Save the new state.
        self.RB = False

    def RBrise(self, gpio, level, tick):
        # Update the count
        if (self.RA):
            self.rcount += 1
        else:
            self.rcount -= 1
        # Save the new state.
        self.RB = True

              
#
#   Main
#
def main(args=None):
    # Initialize the encoder object.
    encoder = Encoder()

    # Loop and read.
    print("Running...")
    try:
        while True:
            print("Encoders:  Left %5d - Right %5d" %
                  (encoder.left(), encoder.right()))
            time.sleep(0.05)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Cleanup (shutting down the callbacks).
    encoder.shutdown()

if __name__ == "__main__":
    main()
