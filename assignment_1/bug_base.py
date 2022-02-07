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

# algorithm implementation  # tested
current_position = start
path = [start]
flag = 1

while computeDistanceTwoPoints(current_position, goal) > step_size:
    dist = np.zeros(len(obstaclesList))
    i = 0
    for obstacle in obstaclesList:
        dist[i] = computeDistancePointToPolygon(obstacle, current_position)
        i = i+1
    if np.min(dist) < step_size:
        flag = 0
        print("Failure: There is an obstacle lying between the start and goal")
        print(path)
        break

    u = (1/computeDistanceTwoPoints(
        current_position, goal))*np.array([goal[0]-current_position[0], goal[1]-current_position[1]])
    current_position = moveROS(client, current_position, step_size, u)    
    path.append(current_position)

if flag:
    path.append(goal)
    print("Success")
    print(path)

outFile = open("output_base.txt", "w")
for element in path:
    outFile.write(str(element[0]) + ", " + str(element[1]) + "\n")
