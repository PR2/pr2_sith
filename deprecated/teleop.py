#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_sith')
import rospy
from pr2_sith.lightsaber import RobotController, ARM_JOINTS
from sensor_msgs.msg import JointState
from joy.msg import Joy
import sys

class Teleop:
    def __init__(self):
        self.rate = 10.0
        self.time = 1.0 / rate

        self.controller = RobotController()
        self.angles = None
        self.button = None

        rospy.Subscriber('/kinect_frames', JointState, self.got_angles)
        rospy.Subscriber('/joy', Joy, self.got_joy)

    def got_angles(self, msg):
        self.angles = msg

    def got_joy(self, msg):
        self.button = msg.buttons[ 15 ] 

    def send_command(self):
        for a in both_arms:
            move = []
            for joint in ARM_JOINTS:
                name = "%s_%s"%(a, joint)
                i = self.angles.name.index(name)
                move.append( self.angles.position[i] )
            goal = controller.make_trajectory(move, self.time, a)
            controller.start_trajectory(goal, a)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.button:
                self.send_command()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('teleop')
    t = Teleop()
    t.spin()

