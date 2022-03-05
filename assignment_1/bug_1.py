#!/usr/bin/env python
import numpy as np
from helper import *
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

rospy.init_node('test', anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# reading files
inFile = open("input.txt", "r")
temp_list = []
read_list = []
for line in inFile.readlines():
    if line == '\n':
        read_list.append(temp_list)
        temp_list = []

    else:
        temp = line.strip().split(",")
        temp = [float(i) for i in temp]
        temp_list.append(list(map(float, temp)))
read_list.append(temp_list)

start = read_list[0][0]
goal = read_list[0][1]
step_size = read_list[0][2][0]
read_list.pop(0)
obstaclesList = read_list

print("Begine bug1")
bug_1 = computeBug1(start, goal, obstaclesList, step_size, client)
path = bug_1[0]
time = bug_1[1]

outFile = open("output_1.txt", "w")
for element in path:
    outFile.write(str(element[0]) + ", " + str(element[1]) + "\n")

outFile2 = open("output_time.txt", "w")
for element2 in time:
    outFile2.write(str(element2) + "\n")


pathPlot(obstaclesList, path, start, goal)
distPlot(path, goal, time)

print("End time:", time[-1])

