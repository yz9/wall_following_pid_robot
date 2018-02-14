#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String, Header
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan #import laserscan msg
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt

    def update_control(self, current_error, reset_prev=False):

        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        #steering angle = P gain + D gain + I gain
        p_gain = self.Kp * self.curr_error

        #i_gain = self.sum_error  + self.Ti * self.curr_error * self.dt
        #self.sum_error = i_gain
        #d_gain = self.Td * self.curr_error_deriv

        # filter
        a = 0.5
        filtered = a * self.prev_error_deriv + (1 - a) * self.curr_error_deriv
        d_gain = self.Td * self.curr_error_deriv
        d_gain = self.Td * filtered
        #PID control
        w = p_gain + d_gain # = control?
        self.control = w

        # update error
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.prev_error_deriv = self.curr_error_deriv
        #print("control", self.control)

    def get_control(self):
        return self.control

class WallFollowerHusky:
    def __init__(self):

        rospy.init_node('wall_follower_husky', anonymous=True)

        #if rospy.has_param("~forward_speed"):
        self.forward_speed = rospy.get_param("~forward_speed")
        #if rospy.has_param("~desired_distance_from_wall"):
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50

        # set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages

        #publisher
        self.cmd_pub = rospy.Publisher('/husky_1/cmd_vel', Twist, queue_size = 100)
        self.cte_pub = rospy.Publisher('/husky_1/cte', Float32, queue_size = 100)

        # set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node

        #subscriber
        self.laser_sub = rospy.Subscriber('/husky_1/scan', LaserScan, self.laser_scan_callback)
        self.pid = PID(1.3, 3, 0.01, 0.2)
        rospy.spin()

    def laser_scan_callback(self, msg):

        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        # If you select option 2, you might want to use cascading PID control.

        # cmd.angular.z = ???
        #rospy.loginfo(msg.angle_min)

        #calculation
        cte = min(msg.ranges) - self.desired_distance_from_wall
        self.cte_pub.publish(cte)
        self.pid.update_control(cte)
        angle = self.pid.get_control()
        linear = Vector3(self.forward_speed, 0, 0)
        angular = Vector3(0, 0, angle)
        twist = Twist(linear, angular)
        self.cmd_pub.publish(twist)
        print("cte : ", cte)
        print("angular z: ", angle)
        print("----")

    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()
