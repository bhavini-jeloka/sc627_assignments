#!/usr/bin/env python
import numpy as np
from math import sqrt, atan2
import matplotlib.pyplot as plt
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib


def computeDistanceTwoPoints(p1, p2):  # tested
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def computeLineThroughTwoPoints(p1, p2):  # tested
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]

    a_prime = (y2 - y1) / (x2 - x1)
    b_prime = -1
    c_prime = (x2 * y1 - x1 * y2) / (x2 - x1)

    nf = (a_prime ** 2 + b_prime ** 2) ** 0.5  # normalisation factor

    a = a_prime / nf
    b = b_prime / nf
    c = c_prime / nf

    return np.array([a, b, c])


def computeDistancePointToLine(q, p1, p2):  # tested
    x1, y1 = q[0], q[1]

    line = computeLineThroughTwoPoints(p1, p2)
    a, b, c = line[0], line[1], line[2]

    return np.abs(a * x1 + b * y1 + c)


def computeDistancePointToSegment(q, p1, p2):   # tested
    vec_p1p2 = [p2[0] - p1[0], p2[1] - p1[1]]
    vec_p2q = [q[0] - p2[0], q[1] - p2[1]]
    vec_p1q = [q[0] - p1[0], q[1] - p1[1]]

    epsilon = 0.00000001

    if np.dot(vec_p1p2, vec_p2q) > epsilon:
        dist = sqrt((q[0] - p2[0]) ** 2 + (q[1] - p2[1]) ** 2)
    elif np.dot(vec_p1p2, vec_p1q) < -epsilon:
        dist = sqrt((q[0] - p1[0]) ** 2 + (q[1] - p1[1]) ** 2)
    else:
        x1 = vec_p1p2[0]
        y1 = vec_p1p2[1]
        x2 = vec_p2q[0]
        y2 = vec_p2q[1]
        dist = abs(x1 * y2 - y1 * x2) / sqrt(x1 * x1 + y1 * y1)

    return dist


def computeDistancePointToPolygon(P, q):    # tested
    n = len(P)
    dist = np.zeros(n)

    for i in range(n - 1):
        dist[i] = computeDistancePointToSegment(q, P[i], P[i + 1])  # dist[i] is the distance between P[i] and P[i+1]

    dist[n - 1] = computeDistancePointToSegment(q, P[n - 1], P[0])  # dist[n-1] is the distance b/w P[n] and P[0]

    return np.min(dist)


def computeLineSegmentForDistancePointToPolygon(P, q):      # tested
    n = len(P)
    dist = np.zeros(n)

    for i in range(n - 1):
        dist[i] = computeDistancePointToSegment(q, P[i], P[i + 1])

    dist[n - 1] = computeDistancePointToSegment(q, P[n - 1], P[0])

    return np.argmin(dist)


def pathPlot(obstaclesList, path):

    plt.figure()
    plt.plot(path[0][0], path[0][1], '-ro')
    plt.plot(path[-1][0], path[-1][1], '-bo')

    for obstacle in obstaclesList:
        obstacle.append(obstacle[0])
        xs, ys = zip(*obstacle)  # create lists of x and y values
        plt.plot(xs, ys)

    X, Y = zip(*path)

    plt.plot(X, Y)
    plt.show()
    return

def gradU(position):
    pass


def moveROS(client, current_position, step_size):
    nextDesired = MoveXYGoal()
    nextDesired.pose_dest.x = current_position[0] + step_size*gradU(current_position[0])
    nextDesired.pose_dest.y = current_position[1] + step_size*gradU(current_position[1])
    nextDesired.pose_dest.theta = atan2(gradU(current_position[0]), gradU(current_position[1]))

    client.send_goal(nextDesired)
    client.wait_for_result()

    nextActual = client.get_result()

    current_position[0] = nextActual.pose_final.x
    current_position[1] = nextActual.pose_final.y
    print(current_position)

    return current_position


def computePotFunc(start, goal, obstaclesList, step_size, client):

    # algorithm implementation
    current_position = start
    path = [start]
    epsilon = 0.00000001

    print("Start PotFunc")

    while np.any(gradU(current_position)) > epsilon:
        current_position = moveROS(client, current_position, step_size)
        path.append(current_position)

    print("Success")
    return path







    





