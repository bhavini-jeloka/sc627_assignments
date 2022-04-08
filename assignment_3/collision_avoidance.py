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


class DynamicPathPlanning:

    def __init__(self):

        self.ANG_MAX = math.pi/18
        self.VEL_MAX = 0.15
        self.epsilon = 0.5
        #self.psuedo_max_iter = 1000

        self.bot_pose = [0.0, 0.0, 0.0]
        self.bot_v = [0.0, 0.0]
        self.bot_omega = 0.0

        self.obs_pose_x = []
        self.obs_pose_y = []
        self.obs_v_x = []
        self.obs_v_y = []

        self.radius = 0.15

        self.goal = [5, 0]

        self.path_x = []
        self.path_y = []
        self.time = []

        self.start_time = time.time()

        # receive data
        rospy.Subscriber('/obs_data', ObsData,
                         self.callback_obs)  # topic name fixed
        rospy.Subscriber('/bot_1/odom', Odometry,
                         self.callback_odom)  # topic name fixed

        # send data
        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=10)
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

        v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x **
                    2 + vel_y ** 2), -self.VEL_MAX), self.VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.bot_pose[0] = data.pose.pose.position.x
        self.bot_pose[1] = data.pose.pose.position.y
        self.bot_pose[2] = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                  data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.bot_v[0] = data.twist.twist.linear.x
        self.bot_v[1] = data.twist.twist.linear.y
        self.bot_omega = data.twist.twist.angular.z

        self.path_x.append(self.bot_pose[0])
        self.path_y.append(self.bot_pose[1])
        self.time.append(time.time()-self.start_time)

    def callback_obs(self, data):
        '''
        Get left robot data
        '''

        self.numObs = len(data.obstacles)

        self.obs_pose_x = np.zeros(self.numObs)
        self.obs_pose_y = np.zeros(self.numObs)
        self.obs_v_x = np.zeros(self.numObs)
        self.obs_v_y = np.zeros(self.numObs)

        for i in range(self.numObs):
            self.obs_pose_x[i] = data.obstacles[i].pose_x
            self.obs_pose_y[i] = data.obstacles[i].pose_y
            self.obs_v_x[i] = data.obstacles[i].vel_x
            self.obs_v_y[i] = data.obstacles[i].vel_y

        # print('left robot')
        # print(data)

    def computeDistanceTwoPoints(self, idxObs):  # tested
        return np.sqrt((self.bot_pose[0] - self.obs_pose_x[idxObs]) ** 2 + (self.bot_pose[1] - self.obs_pose_y[idxObs]) ** 2)

    def angleBetweenVectors(self, v, w):
        v = np.array(v)
        w = np.array(w)
        angle = 0
        if np.linalg.norm(v)*np.linalg.norm(w) != 0.0:
            unit_vector_1 = v / np.linalg.norm(v)
            unit_vector_2 = w / np.linalg.norm(w)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            if abs(dot_product) > 1:
                dot_product = np.sign(dot_product)*1
            angle = np.arccos(dot_product)
        return angle

    def allowableDeviations(self):  # returns the region allowed for the next motion
        heading = self.bot_pose[2]
        minHeading = heading - self.ANG_MAX
        maxHeading = heading + self.ANG_MAX
        possibleHeadings = np.arange(minHeading, maxHeading, math.pi/180)
        possibleMagnitudes = np.arange(0, self.VEL_MAX, 0.01)

        self.deviationCone = []

        for yaw in possibleHeadings:
            for mag in possibleMagnitudes:
                if mag != 0:
                    temp = [mag*np.cos(yaw), mag*np.sin(yaw)]
                    self.deviationCone.append(temp)
        # print(self.deviationCone)

    def velocityObstacleCone(self, idxObs):
        ratio = 2*self.radius/self.computeDistanceTwoPoints(idxObs)
        if abs(ratio) > 1:
            ratio = np.sign(ratio)*1
        sweptAngle = np.arcsin(ratio)
        return sweptAngle

    def inVelocityCone(self, vel_point):
        numObs = len(self.obs_pose_x)
        for i in range(numObs):
            vel_point_new = np.array(
                [vel_point[0]-self.obs_v_x[i], vel_point[1]-self.obs_v_y[i]])
            BA = np.array([self.obs_pose_x[i]-self.bot_pose[0],
                          self.obs_pose_y[i]-self.bot_pose[1]])
            if self.angleBetweenVectors(vel_point_new, BA) < self.velocityObstacleCone(i):
                print(self.bot_pose, i, "True")
                return True

    def noCollisionPoints(self):
        self.noCollision = []
        self.allowableDeviations()
        for point in self.deviationCone:
            if not self.inVelocityCone(point):
                self.noCollision.append(point)

    def heuristic(self):
        self.noCollisionPoints()
        deviationFromGoal = []
        direction2goal = [self.goal[0]-self.bot_pose[0],
                          self.goal[1]-self.bot_pose[1]]
        for point in self.noCollision:
            deviationFromGoal.append(
                self.angleBetweenVectors(direction2goal, point))
        i = np.argmin(deviationFromGoal)
        return self.noCollision[i]

    def goalNotReached(self):
        if np.sqrt((self.bot_pose[0] - self.goal[0]) ** 2 + (self.bot_pose[1] - self.goal[1]) ** 2) > self.epsilon:
            return True
        else:
            return False

    def BotMotion(self):

        while self.goalNotReached():
            #print("Test 1")
            cmd_x, cmd_y = self.heuristic()
            #print("Test 2")
            v_lin, v_ang = self.velocity_convert(
                None, None, self.bot_pose[2], cmd_x, cmd_y)
            #print("Test 3")

            # publish the velocities below
            vel_msg = Twist()
            vel_msg.linear.x = v_lin
            vel_msg.angular.z = v_ang
            self.pub_vel.publish(vel_msg)
            self.r.sleep()
            #print("Test 4")


if __name__ == '__main__':
    rospy.init_node('assign3_skeleton', anonymous=True)
    robot = DynamicPathPlanning()
    robot.BotMotion()

    fig1, ax1 = plt.subplots(1, 1, figsize=(
        14, 14), sharex=True, gridspec_kw={'hspace': 0.2})
    ax1.plot(robot.path_x, robot.path_y)
    ax1.set_ylabel('y-axis')
    ax1.set_xlabel('x-axis')
    ax1.set_title('Path taken by the robot')
    ax1.grid()
    plt.show()

    fig2, ax2 = plt.subplots(1, 1, figsize=(
        14, 14), sharex=True, gridspec_kw={'hspace': 0.2})
    ax2.plot(robot.time, robot.path_x)
    ax2.set_ylabel('X Coordinate')
    ax2.set_xlabel('Time')
    ax2.set_title('Position vs. Time (X)')
    ax2.grid()
    plt.show()
