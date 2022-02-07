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


def isCounterClockwise(v, w):   # tested
    if v[0] * w[1] - v[1] * w[0] > 0:
        return True
    else:
        return False


def computeTangentVectorToPolygon(P, q):
    n = len(P)
    idx = computeLineSegmentForDistancePointToPolygon(P, q)

    if idx == n - 1:
        p1 = P[n-1]
        p2 = P[0]
        p3 = P[1]
    elif idx == n-2:
        p1 = P[n-2]
        p2 = P[n-1]
        p3 = P[0]
    else:
        p1 = P[idx]
        p2 = P[idx + 1]
        p3 = P[idx + 2]

    vec_p1p2 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    vec_p2p3 = np.array([p3[0] - p2[0], p3[1] - p2[1]])

    vec_p1q = np.array([q[0] - p1[0], q[1] - p1[1]])
    vec_p2q = np.array([q[0] - p2[0], q[1] - p2[1]])

    epsilon = 0.00000001

    if np.dot(vec_p1p2, vec_p2q) > epsilon:
        u = 0.01*computeLineThroughTwoPoints(q, p2)[0:2]
        if isCounterClockwise(vec_p1p2, u) == False:
            u = -1*u

    elif np.dot(vec_p1p2, vec_p1q) < -epsilon:
        u = 0.01*computeLineThroughTwoPoints(q, p1)[0:2]
        if isCounterClockwise(-1*vec_p1p2, u) == False:
            u = -1*u

    else:
        x1 = vec_p1p2[0]
        y1 = vec_p1p2[1]
        mod = sqrt(x1 * x1 + y1 * y1)

        if isCounterClockwise(vec_p1p2, vec_p2p3):
            u = (1 / mod) * np.array([x1, y1])
        else:
            u = -(1 / mod) * np.array([x1, y1])

    return u


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


def moveROS(client, current_position, step_size, u):
    nextDesired = MoveXYGoal()
    nextDesired.pose_dest.x = current_position[0] + step_size*u[0]
    nextDesired.pose_dest.y = current_position[1] + step_size*u[1]
    nextDesired.pose_dest.theta = atan2(u[1], u[0])

    client.send_goal(nextDesired)
    client.wait_for_result()

    nextActual = client.get_result()

    current_position[0] = nextActual.pose_final.x
    current_position[1] = nextActual.pose_final.y

    return current_position


def computeBug1(start, goal, obstaclesList, step_size, client):

    # algorithm implementation
    current_position = start
    path = [start]

    print("Start bug1")

    while computeDistanceTwoPoints(current_position, goal) > step_size:
        dist = np.zeros(len(obstaclesList))
        i = 0

        print("Inside global while loop")

        for obstacle in obstaclesList:
            dist[i] = computeDistancePointToPolygon(obstacle, current_position)
            i = i + 1

        P = obstaclesList[np.argmin(dist)]

        if np.min(dist) < step_size:
            bug1_start = current_position
            bug1_dist = []
            bug1_dist.append(computeDistanceTwoPoints(current_position, goal))
            path.append(current_position)
        
            print("Took first step")

            while (computeDistanceTwoPoints(current_position, bug1_start) > 0.99*step_size or len(bug1_dist) < 2):
                print("Inside circumnav")
                u = computeTangentVectorToPolygon(P, current_position)
                current_position = moveROS(client, current_position, step_size, u)
                path.append(current_position)
                bug1_dist.append(computeDistanceTwoPoints(current_position, goal))
            
            print("Circumnav done")

            u = (1 / computeDistanceTwoPoints(current_position, bug1_start)) * \
                               np.array([bug1_start[0] - current_position[0],
                                         bug1_start[1] - current_position[1]])

            current_position = moveROS(client, current_position, step_size, u)
            path.append(current_position)

            closestGoal = np.argmin(bug1_dist)

            for position in range(closestGoal):  # see where you land
                print("moving to closest obs point")
                u = computeTangentVectorToPolygon(P, current_position)
                current_position = moveROS(client, current_position, step_size, u)
                path.append(current_position)

            u = (1 / computeDistanceTwoPoints(current_position, goal)) * np.array([
                goal[0] - current_position[0], goal[1] - current_position[1]])
            current_position = moveROS(client, current_position, step_size, u)
            path.append(current_position)

            if computeDistancePointToPolygon(P, current_position) < np.min(dist):
                print("Failure")
                return path

        else:
            print("moving towards goal")
            u =  (1 / computeDistanceTwoPoints(current_position, goal)) * np.array([
                goal[0] - current_position[0], goal[1] - current_position[1]])
            current_position = moveROS(client, current_position, step_size, u)
            path.append(current_position)

    path.append(goal)
    print("Success")
    return path







    





