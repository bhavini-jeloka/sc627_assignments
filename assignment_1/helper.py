#!/usr/bin/env python
import numpy as np
from math import sqrt, atan2
import matplotlib.pyplot as plt
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import copy
import time


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
        # dist[i] is the distance between P[i] and P[i+1]
        dist[i] = computeDistancePointToSegment(q, P[i], P[i + 1])

    # dist[n-1] is the distance b/w P[n] and P[0]
    dist[n - 1] = computeDistancePointToSegment(q, P[n - 1], P[0])

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
        u = computeLineThroughTwoPoints(q, p2)[0:2]
        if isCounterClockwise(vec_p1p2, u) == False:
            u = -1*u

    elif np.dot(vec_p1p2, vec_p1q) < -epsilon:
        u = computeLineThroughTwoPoints(q, p1)[0:2]
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


def pathPlot(obstaclesList, path, start, goal):
    plt.figure()

    #plt.scatter(start[0], start[1])
    #plt.scatter(goal[0], goal[1])

    for obstacle in obstaclesList:
        obstacle.append(obstacle[0])
        xs, ys = zip(*obstacle)  # create lists of x and y values
        plt.plot(xs, ys)
    
    X, Y = zip(*path)
    plt.plot(X, Y, '-.')
    plt.show()
    return


def distPlot(path, goal, time):
    distance2goal = []
    for point in path:
        distance2goal.append(computeDistanceTwoPoints(point, goal))

    plt.figure()
    plt.plot(time, distance2goal, '-.')
    plt.xlabel("Time")
    plt.ylabel("Distance to the Goal")
    plt.show()
    return


def pathLength(path):
    n = len(path)
    sum = 0
    for i in range(n-1):
        sum = sum + computeDistanceTwoPoints(path[i], path[i+1])
    return sum


def centroid(P):
    return np.mean(P, axis=0)


def angleVec(v, w):
    v = np.array(v)
    w = np.array(w)
    angle = 0
    if np.linalg.norm(v)*np.linalg.norm(w) != 0.0:
        unit_vector_1 = v / np.linalg.norm(v)
        unit_vector_2 = w / np.linalg.norm(w)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        #if abs(dot_product) > 1:
        #    dot_product = np.sign(dot_product)*1
        angle = np.arccos(dot_product)
    return angle


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
    path = []
    time_taken = []

    start_time = time.time()
    time_taken.append(start_time-start_time)
    path.append(current_position[:])

    while computeDistanceTwoPoints(current_position, goal) > step_size:
        dist = np.zeros(len(obstaclesList))
        i = 0

        for obstacle in obstaclesList:
            dist[i] = computeDistancePointToPolygon(obstacle, current_position)
            i = i + 1

        P = obstaclesList[np.argmin(dist)]

        if np.min(dist) < step_size:
            bug1_start = current_position[:]
            bug1_dist = []
            bug1_point = []
            bug1_dist.append(computeDistanceTwoPoints(current_position, goal))
            bug1_point.append(current_position)

            path.append(current_position[:])
            time_taken.append(time.time()-start_time)

            centre = centroid(P)
            centre2start = [bug1_start[0]-centre[0], bug1_start[1]-centre[1]]

            while angleVec([current_position[0]-centre[0], current_position[1]-centre[1]], centre2start) > step_size or len(bug1_dist) < 25:
                u = computeTangentVectorToPolygon(P, current_position)
                current_position = moveROS(
                    client, current_position, step_size, u)
                path.append(current_position[:])
                time_taken.append(time.time()-start_time)
                bug1_dist.append(computeDistanceTwoPoints(
                    current_position, goal))
                bug1_point.append(current_position[:])

            u = (1 / computeDistanceTwoPoints(current_position, bug1_start)) * \
                np.array([bug1_start[0] - current_position[0],
                          bug1_start[1] - current_position[1]])

            current_position = moveROS(client, current_position, step_size, u)
            path.append(current_position[:])
            time_taken.append(time.time()-start_time)

            closestGoal = bug1_point[np.argmin(bug1_dist)]

            centre2closestGoal = [closestGoal[0] -
                                  centre[0], closestGoal[1]-centre[1]]

            while angleVec([current_position[0]-centre[0], current_position[1]-centre[1]], centre2closestGoal) > step_size or len(bug1_dist) < 10:
                u = computeTangentVectorToPolygon(P, current_position)
                current_position = moveROS(
                    client, current_position, step_size, u)
                path.append(current_position[:])
                time_taken.append(time.time()-start_time)

            u = (1 / computeDistanceTwoPoints(current_position, goal)) * np.array([
                goal[0] - current_position[0], goal[1] - current_position[1]])

            current_position = moveROS(client, current_position, step_size, u)
            path.append(current_position[:])
            time_taken.append(time.time()-start_time)

            if computeDistancePointToPolygon(P, current_position) < np.min(dist):
                print("Failure")
                return [path, time_taken]

        else:
            u = (1 / computeDistanceTwoPoints(current_position, goal)) * np.array([
                goal[0] - current_position[0], goal[1] - current_position[1]])
            current_position = moveROS(client, current_position, step_size, u)
            path.append(current_position[:])
            time_taken.append(time.time()-start_time)

    time_taken.append(time.time()-start_time)
    path.append(current_position[:])
    print("Success")
    return [path, time_taken]
