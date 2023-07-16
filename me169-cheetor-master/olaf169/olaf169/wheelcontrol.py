#!/usr/bin/env python3
#
#   wheelcontrol_skeleton.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#
import rclpy
import traceback

from math import pi

from olaf169.encoder import Encoder
from olaf169.driver2  import Driver
from olaf169.gyro import Gyro
import smbus

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
import time
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType

#
#   Constants
#
# Left/right definitions
LEFT   = 0
RIGHT  = 1
MOTORS = 2

LEFTNAME  = 'leftwheel'
RIGHTNAME = 'rightwheel'
GYRO = 'gyro'
MOTORNUM  = {LEFTNAME:LEFT, RIGHTNAME:RIGHT}
assert MOTORS == len(MOTORNUM)

# Desired loop rates
SERVORATE    = 200.0            
PUBLISHRATE  =  50.0
PUBLISHCYCLE = int(SERVORATE / PUBLISHRATE)

MAX_VEL = 25.0 # rad/s
MIN_VEL = -MAX_VEL
MAX_ACC = 60.0 # rad/s2 # FAST BOT - CHANGED 4/20 - "It's a lot more crispier now tho" - Krish

#
#   Utilities
#
def sat(x, limit):
    # Saturate the value to +/- the limit.
    return min(max(x, -limit), limit)

def inflate(x, offset):
    # Add the offset in each direction, leaving just a tiny deadband.
    if   (x >  1.0): return (x + offset)
    elif (x < -1.0): return (x - offset)
    else:            return (x)

#
#   Wheel Control Node Class
#
class WheelControlNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Incoming commands.
        self.cmdvel  = [0.0, 0.0]
        self.cmdtime = [self.get_clock().now(), self.get_clock().now()]

        self.pubcount = 0               # Publishing counter

        i2cbus = smbus.SMBus(1)
        # Initialize the I/O objects for the encoders/gyro/motors.
        self.encoder = Encoder()
        self.gyro = Gyro(i2cbus)
        self.driver = Driver(i2cbus)

        # Initialize any other state variables.
        self.pos = {LEFT: self.encoder.left() * 2 * pi / 720, RIGHT: self.encoder.right() * 2 * pi / 720}
        self.pos_des = {LEFT: 0, RIGHT: 0}
        self.pwm_des = {LEFT:0, RIGHT:0}
        self.vel_des = {LEFT:0, RIGHT: 0}
        self.vel = {LEFT: 0, RIGHT: 0}

        # get robot name
        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)
        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value = '')
        param  = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        # Constants
        if self.prefix == 'robot1':
            self.Tfilter = 0.20 # 0.15
            self.Tfeedback = 0.20 # 0.15
            self.error_max = 10 # 10
            self.pwm_slope = 1/0.104 #### INSERT CALCULATED VALUE
            self.pwm_intercept_straight = 30 # 3.81/0.104 #### INSERT CALCULATED VALUE
            self.pwm_intercept_turn = 70 # 3.81/0.0789 #### INSERT CALCULATED VALUE

        else:
            self.Tfilter = 0.18
            self.Tfeedback = 0.18
            self.error_max = 10
            self.pwm_slope = 1/0.104 #### INSERT CALCULATED VALUE
            self.pwm_intercept_straight = 25 # 3.81/0.104 #### INSERT CALCULATED VALUE
            self.pwm_intercept_turn = 38 # 3.81/0.0789 #### INSERT CALCULATED VALUE
        
        self.omega = 0
        self.theta = 0
        self.prev_time = time.time()
        self.thetaOld = 0
        self.count = 0 
        # self.vels = {LEFT: [], RIGHT: []}

        # Create the publishers for the actual and (internal) desired.
        self.pubdes = self.create_publisher(JointState, 'wheel_desired', 10)
        self.pubact = self.create_publisher(JointState, 'wheel_state',   10)

        # Create a subscriber to listen to wheel commands.
        self.subcmd = self.create_subscription(
            JointState, 'wheel_command', self.cb_cmdmsg, 10)

        # Create the timer to drive the node.
        self.timer = self.create_timer(1/SERVORATE, self.cb_timer)
        self.dt    = self.timer.timer_period_ns * 1e-9

        # Report and return.
        self.get_logger().info("Wheel control running with dt %fs (%fHz)" %
                               (self.dt, 1.0/self.dt))

    # Shutdown
    def shutdown(self):
        # Destroy the timer.
        self.timer.destroy()

        # Clean up the low level.
        self.driver.shutdown()
        self.encoder.shutdown()

        # Finally, shut down the node.
        self.destroy_node()

    # Command subscription callback
    def cb_cmdmsg(self, msg):
        # Check the message structure.
        assert len(msg.name) == len(msg.velocity), \
            "Wheel command msg name/velocity must have same length"

        # Note the current time (to timeout the command).
        now = self.get_clock().now()

        # Extract and save the velocity commands at this time.
        for i in range(len(msg.name)):
            mtr = MOTORNUM[msg.name[i]]
            self.cmdvel[mtr]  = msg.velocity[i]
            self.cmdtime[mtr] = now

    # Timer callback
    def cb_timer(self):
        # Grab the current time.
        now = self.get_clock().now()
        dt_sec = 1 / SERVORATE

        # There is a lot of work to be done here!!!
        self.prev_pos = self.pos.copy()

        # Get motor position (radians) from encoder readings
        self.pos[LEFT] = self.encoder.left() * 2 * pi / 720
        self.pos[RIGHT] = self.encoder.right() * 2 * pi / 720

        # Get (filtered) velocity from change in motor position
        vel_raw = {LEFT:0, RIGHT:0}
        vel_raw[LEFT] = (self.pos[LEFT] - self.prev_pos[LEFT]) / dt_sec
        vel_raw[RIGHT] = (self.pos[RIGHT] - self.prev_pos[RIGHT]) / dt_sec
        self.vel[LEFT] = self.vel[LEFT] + dt_sec / self.Tfilter * (vel_raw[LEFT] - self.vel[LEFT])
        self.vel[RIGHT] = self.vel[RIGHT] + dt_sec / self.Tfilter * (vel_raw[RIGHT] - self.vel[RIGHT])

        # Save velocities
        # self.vels[LEFT].append(self.vel[LEFT])
        # self.vels[RIGHT].append(self.vel[RIGHT])`

        # If last command was more than 0.25 seconds ago, set velocity commands to 0
        time_since_command = now - self.cmdtime[LEFT] # Left is arbitrary
        if time_since_command.nanoseconds / (10 ** 9) > 0.25:
            self.cmdvel[LEFT] = 0.0
            self.cmdvel[RIGHT] = 0.0

        # Save vel_des from previous iteration before we update vel_des
        prev_vel_des = self.vel_des.copy()

        # Cap command velocity
        self.vel_des[LEFT] = max(MIN_VEL, min(self.cmdvel[LEFT], MAX_VEL))
        self.vel_des[RIGHT] = max(MIN_VEL, min(self.cmdvel[RIGHT], MAX_VEL))
        
        # Lambda to determine sign of x
        sign = lambda x: 1 if x >= 0 else -1

        # avgVelLeft = sum(self.vels[LEFT][-20:]) / 20
        # avgVelRight = sum(self.vels[RIGHT][-20:]) / 20
        
        # Compute acceleration from the previous command
        acc_des = {LEFT: 0, RIGHT: 0}
        acc_des[LEFT] = (self.vel_des[LEFT] - prev_vel_des[LEFT]) / dt_sec
        acc_des[RIGHT] = (self.vel_des[RIGHT] - prev_vel_des[RIGHT]) / dt_sec

        # Cap acceleration and velocity if needed
        if abs(acc_des[LEFT]) > MAX_ACC:
            acc_des[LEFT] = MAX_ACC * sign(acc_des[LEFT])
            self.vel_des[LEFT] = prev_vel_des[LEFT] + acc_des[LEFT] * dt_sec
        
        if abs(acc_des[RIGHT]) > MAX_ACC:
            acc_des[RIGHT] = MAX_ACC * sign(acc_des[RIGHT])
            self.vel_des[RIGHT] = prev_vel_des[RIGHT] + acc_des[RIGHT] * dt_sec
        
        # Integrate vel_des to find pos_des
        self.pos_des[LEFT] = self.pos_des[LEFT] + self.vel_des[LEFT] * dt_sec
        self.pos_des[RIGHT] = self.pos_des[RIGHT] + self.vel_des[RIGHT] * dt_sec

        self.pos_des[LEFT] = min(max(self.pos_des[LEFT], self.pos[LEFT] - self.error_max), self.pos[LEFT] + self.error_max)
        self.pos_des[RIGHT] = min(max(self.pos_des[RIGHT], self.pos[RIGHT] - self.error_max), self.pos[RIGHT] + self.error_max)

        v_fin = {LEFT:0, RIGHT:0}
        v_fin[LEFT] = self.vel_des[LEFT] + (self.pos_des[LEFT] - self.pos[LEFT]) / self.Tfeedback
        v_fin[RIGHT] = self.vel_des[RIGHT] + (self.pos_des[RIGHT] - self.pos[RIGHT]) / self.Tfeedback

        # Compute PWM values from vel_des
        self.pwm_des[LEFT] = self.pwm_slope * v_fin[LEFT]
        if abs(v_fin[LEFT]) >= 0:
            # Add the proportional part no matter what, include the intercept only if
            # large enough velocity
            if self.vel_des[LEFT] * self.vel_des[RIGHT] <= 0:
                self.pwm_des[LEFT] += sign(v_fin[LEFT]) * self.pwm_intercept_straight
            else:
                self.pwm_des[LEFT] += sign(v_fin[LEFT]) * self.pwm_intercept_turn
        else:
            # Set to 0.0 to ignore the intercept
            self.pwm_des[LEFT] = 0.0

        self.pwm_des[RIGHT] = self.pwm_slope * v_fin[RIGHT]
        if abs(v_fin[RIGHT]) >= 0:
            if self.vel_des[LEFT] * self.vel_des[RIGHT] <= 0:
                self.pwm_des[RIGHT] += sign(v_fin[RIGHT]) * self.pwm_intercept_straight
            else:
                self.pwm_des[RIGHT] += sign(v_fin[RIGHT]) * self.pwm_intercept_turn
        else:
            self.pwm_des[RIGHT] = 0.0

        # Send the PWM.
        self.driver.pwm(self.pwm_des[LEFT], self.pwm_des[RIGHT])

        # Update Gyro 
        curr_time = time.time()
        (self.omega, _) = self.gyro.read()
        self.theta += (curr_time - self.prev_time) * self.omega
        self.prev_time = curr_time

        self.count += 1
        self.thetaOld += self.dt * self.omega
        
        # Publish the actual and desired wheel states
        self.pubcount = (self.pubcount + 1) % PUBLISHCYCLE
        if not self.pubcount:
            msg = JointState()
            msg.header.stamp = now.to_msg()
            msg.name         = [LEFTNAME,  RIGHTNAME, GYRO]

            msg.position     = [self.pos[LEFT], self.pos[RIGHT], self.theta]
            msg.velocity     = [self.vel[LEFT], self.vel[RIGHT], self.omega]
            msg.effort       = []
            self.pubact.publish(msg)

            msg.name         = [LEFTNAME,  RIGHTNAME]
            msg.position     = [self.pos_des[LEFT], self.pos_des[RIGHT]]
            msg.velocity     = [self.vel_des[LEFT], self.vel_des[RIGHT]]
            msg.effort       = [self.pwm_des[LEFT], self.pwm_des[RIGHT]]
            self.pubdes.publish(msg)

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = WheelControlNode('wheelcontrol')

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
