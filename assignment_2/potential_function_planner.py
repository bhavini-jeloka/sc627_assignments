#!/usr/bin/env python
import numpy as np
from helper import *
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

rospy.init_node('test', anonymous= True)

#Initialize client
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

path, desired_path = computePotFunc(start, goal, obstaclesList, step_size, client, 2, 0.8, 0.8, 2*np.ones(len(obstaclesList)))

outFile = open("output.txt", "w")
outFile2 = open("output2.txt", "w")

for i in range(len(path)):
    outFile.write(str(path[i][0]) + ", " + str(path[i][1]) + "\n")
    outFile2.write(str(desired_path[i][0]) + ", " + str(desired_path[i][1]) + "\n")

pathPlot(obstaclesList, path, goal, desired_path)

# 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
