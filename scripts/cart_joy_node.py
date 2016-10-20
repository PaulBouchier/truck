#!/usr/bin/env python

# Node for the big truck gazebo model
########################################################################
# Copyright (c) 2010 University at Albany. All right reserved.
# Derived work Copyright (c) 2014 Paul Bouchier. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#######################################################################
#
#                            DISCLAIMER
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


"""
ROS node for big truck gazebo model
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"
__author__ = "paul.bouchier@gmail.com (Paul Bouchier)"

import roslib;
import rospy
from math import sin,cos

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class TruckSimNode:

    def __init__(self):
        """ Start up connection to the joystick & gazebo model. """
        rospy.init_node('TruckSimNode')

        self.steer_angle_topic = rospy.get_param('~steer_angle_topic', "steer_angle")
        self.chassis_force_topic = rospy.get_param('~chassis_force_topic', "chassis_force")

        rospy.Subscriber("joy", Joy, self.joyCb)

        self.steer_pub = rospy.Publisher(self.steer_angle_topic, Float64, queue_size=1)
        self.chassis_force_pub = rospy.Publisher(self.chassis_force_topic, Float64, queue_size=1)

        # array of joy buttons/axes:
        # 0: turn - (+ve = left)
        # 1: acceleration (+ve = forward)
        self.joy_buttons = [0, 0]
        self.steer_joint = Float64()
        self.chassis_force = Float64()

    def spin(self):        
        # main loop of node
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            # send updated movement commands
            self.steer_joint.data = 0.45 * self.joy_buttons[0]
            self.steer_pub.publish(self.steer_joint)

            self.chassis_force.data = self.joy_buttons[1]
            self.chassis_force_pub.publish(self.chassis_force)

            # wait, then do it again
            r.sleep()

    """ Receive & store joystick controls """
    def joyCb(self,req):
        self.joy_buttons[0] = req.axes[0] # left joystick left/right
        self.joy_buttons[1] = req.axes[1] # left joystick forward/back
        # print 'turn: ', self.joy_buttons[0], ' accel: ', self.joy_buttons[1]

if __name__ == "__main__":    
    robot = TruckSimNode()
    robot.spin()
    
