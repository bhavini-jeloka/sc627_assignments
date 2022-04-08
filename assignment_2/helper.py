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

    if x2-x1==0:
        a = 1
        b = 0
        c = -x1

    else:
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


def dropPerpendicularToPolygon(P, q):
    n = len(P)
    idx = computeLineSegmentForDistancePointToPolygon(P, q)

    if idx == n - 1:
        p1 = P[n - 1]
        p2 = P[0]
    else:
        p1 = P[idx]
        p2 = P[idx + 1]

    vec_p1p2 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    vec_p1q = np.array([q[0] - p1[0], q[1] - p1[1]])
    vec_p2q = np.array([q[0] - p2[0], q[1] - p2[1]])

    epsilon = 0.00000001

    if np.dot(vec_p1p2, vec_p2q) > epsilon:
        return p2

    elif np.dot(vec_p1p2, vec_p1q) < -epsilon:
        return p1

    else:
        a, b, c = computeLineThroughTwoPoints(p1, p2)
        c1 = -b * q[0] + a * q[1]
        c_x = -a*c - b*c1
        c_y = -b*c + a*c1
        return np.array([c_x, c_y])


def pathPlot(obstaclesList, path, goal, desired_path):
    plt.figure()
    plt.plot(path[0][0], path[0][1], '-ro')
    plt.plot(path[-1][0], path[-1][1], '-ro')
    plt.plot(goal[0], goal[1], '-bo')

    for obstacle in obstaclesList:
        obstacle.append(obstacle[0])
        xs, ys = zip(*obstacle)  # create lists of x and y values
        plt.plot(xs, ys)

    X, Y = zip(*path)
    plt.plot(X, Y, '--', label ='Actual Path')

    Xd, Yd = zip(*desired_path)
    plt.plot(Xd, Yd, '-.', label ='Desired Path')

    plt.title("Path Followed by the Robot")
    plt.xlabel("X")
    plt.ylabel("Y")   
    plt.legend()
    
    plt.show()
    return


def gradU(q, q_goal, d_star, x, eta, q_i_star, obstaclesList):

    if computeDistanceTwoPoints(q, q_goal) > d_star:
        gradientAttractive = d_star * x * (np.array(q_goal) - np.array(q)) / computeDistanceTwoPoints(q, q_goal)
    else:
        gradientAttractive = x * (np.array(q_goal)-np.array(q))

    n = len(obstaclesList)
    gradientRepulsive = np.zeros(n)

    for i in range(n):
        dist2P = computeDistancePointToPolygon(obstaclesList[i], q)
        if not dist2P > q_i_star[i]:
            c = dropPerpendicularToPolygon(obstaclesList[i], q)
            grad_obstacle = (np.array(q) - np.array(c)) / dist2P
            gradientRepulsive = gradientRepulsive + eta * ((1 / dist2P) - (1 / q_i_star[i])) * (
                        1 / dist2P ** 2) * grad_obstacle

    return gradientAttractive + gradientRepulsive 


def moveROS(client, current_position, step_size, grad):

    nextDesired = MoveXYGoal()
    nextDesired.pose_dest.x = current_position[0] + step_size*grad[0]
    nextDesired.pose_dest.y = current_position[1] + step_size*grad[1]
    nextDesired.pose_dest.theta = atan2(grad[1], grad[0])

    client.send_goal(nextDesired)
    client.wait_for_result()

    nextActual = client.get_result()

    desired_position = [nextDesired.pose_dest.x, nextDesired.pose_dest.y]

    current_position[0] = nextActual.pose_final.x
    current_position[1] = nextActual.pose_final.y

    return current_position, desired_position


def computePotFunc(start, goal, obstaclesList, step_size, client, d_star, x, eta, q_i_star):

    # algorithm implementation
    current_position = start
    path = []
    desired_path = []
    epsilon = 0.2970

    path.append(start[:])
    desired_path.append(start[:])

    grad = gradU(current_position, goal, d_star, x, eta, q_i_star, obstaclesList)

    while np.linalg.norm(grad) > epsilon:
        current_position, desired_position = moveROS(client, current_position, step_size, grad)
        path.append(current_position[:])
        desired_path.append(desired_position[:]) 
        grad = gradU(current_position, goal, d_star, x, eta, q_i_star, obstaclesList)

    return path, desired_path







    





