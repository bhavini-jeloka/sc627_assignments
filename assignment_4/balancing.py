#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import matplotlib.pyplot as plt

class Coordination:

    def __init__(self):

        self.ANG_MAX = math.pi/18
        self.VEL_MAX = 0.15
        self.epsilon = 0.001
        self.psuedo_max_iter = 1000

        self.coord = [0.0, 0.0, 0.0]
        self.coord_left = [0.0, 0.0, 0.0]
        self.coord_right = [0.0, 0.0, 0.0]

        self.v = 0.0
        self.v_left = 0.0
        self.v_right = 0.0

        self.omega = 0.0

        self.path_x = []
        self.path_y = []
        self.time = []
        
        self.start_time = time.time()

        # receive data
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        rospy.Subscriber('/left_odom', Odometry, self.callback_left_odom)
        rospy.Subscriber('/right_odom', Odometry, self.callback_right_odom)

        # send data
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.r = rospy.Rate(30)

    def velocity_convert(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 1  # modify if necessary

        ang = math.atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * math.pi

        ang_err = min(max(ang - theta, -self.ANG_MAX), self.ANG_MAX)

        v_lin = min(max(vel_x, -self.VEL_MAX), self.VEL_MAX)
        v_ang = 0.0
        return v_lin, v_ang

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.coord[0] = data.pose.pose.position.x
        self.coord[1] = data.pose.pose.position.y
        self.v = data.twist.twist.linear.x
        self.omega = data.twist.twist.angular.z
        self.coord[2] = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                               data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.path_x.append(self.coord[0])
        self.path_y.append(self.coord[1])
        self.time.append(time.time()-self.start_time)

    def callback_left_odom(self, data):
        '''
        Get left robot data
        '''

        self.coord_left[0] = data.pose.pose.position.x
        self.coord_left[1] = data.pose.pose.position.y
        self.v_left = data.twist.twist.linear.x
        self.coord_left[2] = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

        # print('left robot')
        # print(data)

    def callback_right_odom(self, data):
        '''
        Get right robot data
        '''

        self.coord_right[0] = data.pose.pose.position.x
        self.coord_right[1] = data.pose.pose.position.y
        self.v_right = data.twist.twist.linear.x
        self.coord_right[2] = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                     data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

        # print('right robot')
        # print(data)

    def BalanceBot(self):
        while not rospy.is_shutdown():  # replace with balancing reached?

            # get odom data, left_odom and right_odom

            # calculate v_x, v_y as per the balancing strategy

            k = 1
            u = k*(self.coord_left[0]-self.coord[0]) + \
                k*(self.coord_right[0]-self.coord[0])
            theta = math.pi if np.sign(u) < 0 else float(0)

            # Make sure your velocity vector is feasible (magnitude and direction)
            cmd_v_x = u
            cmd_v_y = 0

            # convert velocity vector to linear and angular velocties using velocity_convert function given above
            v_lin, v_ang = self.velocity_convert(
                None, None, theta, cmd_v_x, cmd_v_y)

            # publish the velocities below
            vel_msg = Twist()
            vel_msg.linear.x = v_lin
            vel_msg.angular.z = v_ang
            self.pub_vel.publish(vel_msg)
            self.r.sleep()

            pseudo_iter = 0
            if abs(self.v)<self.epsilon and abs(self.v_left)<self.epsilon and abs(self.v_right)<self.epsilon and pseudo_iter>self.psuedo_max_iter:
                break

            pseudo_iter += 1

            # store robot path with time stamps (data available in odom topic)


if __name__ == '__main__':
    rospy.init_node('assign4_skeleton', anonymous=True)
    robot = Coordination()
    robot.BalanceBot()

    fig1, ax1 = plt.subplots(1, 1, figsize=(14, 14), sharex=True, gridspec_kw={'hspace': 0.2})
    ax1.plot(robot.path_x, robot.path_y)
    ax1.set_ylabel('y-axis')
    ax1.set_xlabel('x-axis')
    ax1.set_title('Path taken by the robot')
    ax1.grid()

    fig2, ax2 = plt.subplots(1, 1, figsize=(14, 14), sharex=True, gridspec_kw={'hspace': 0.2})
    ax2.plot(robot.time, robot.path_x)
    ax2.set_ylabel('X Coordinate')
    ax2.set_xlabel('Time')
    ax2.set_title('Position vs. Time (X)')
    ax2.grid()


