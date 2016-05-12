#!/usr/bin/env python
from __future__ import print_function
import time
import random
import numpy as np
import numpy.linalg as la

import roslib; roslib.load_manifest('team_wpi_integrator_chains')
import rospy
from std_msgs.msg import Header
from fmrb import integrator_chains

import heapq
import math

def distance(pos1, pos2):
    return la.norm(np.asarray(pos1) - np.asarray(pos2))

def in_polytope(current, polytope, threshold):
    inside = False
    bbox = polytope.get_bbox()
    for i in range(0, len(current)):
        if  abs(bbox[2*i + 1] - (bbox[2*i]) < 2*abs(threshold)):
            threshold = 0;
    coll = 0
    # Check if each dimension of current is in the polytope
    for i in range(0, len(current)):
        if current[i] > (bbox[2*i] - threshold) and current[i] < (bbox[2*i + 1] + threshold):
            coll = coll + 1        
        if coll == len(current):
            inside = True
            break
    return inside

def in_polytopes(current, polytopes, threshold):
    inside = False
    for polytope in polytopes:
        if not inside:
            if in_polytope(current, polytope, threshold):
                inside = True
                break
    return inside

def in_collision(current, obstacles, threshold):
    threshold = 0.3
    return in_polytopes(current, obstacles, threshold)

def in_goals(current, goals, threshold):
    threshold = -0.2
    return in_polytopes(current, goals, threshold)

def in_goal(current, index, goals, threshold):
    return in_goals(current, [goals[index]], threshold)

def goalsRealizable(goals, obstacles):
    areThey = True
    for goal in goals:
        if encased(goal, obstacles):
            areThey = False
            break
    return areThey

def encased(goal, obstacles):
    isIt = False
    polyV = goal.getVrep()
    center = np.mean(polyV, axis=0)
    if in_collision(center, obstacles, 0): # Check the middle
        coll = 0
        dim = len(center)
        isFeasible = True
        bbox = goal.get_bbox()
        for i in range(0, int(2^dim)): # Check the corners
            binary = decToBin(i, dim)
            point = []
            for j in range(0, dim):
                dim_point = bbox[2*j + binary[j]]
                point.append(dim_point)
            if in_collision(point, obstacles, 0):
                coll = coll + 1
        if coll == int(2^dim):
            isIt = True
    return isIt

# Outputs valid targets in each goal as a numpy array
def goal_targets(goals, obstacles):
    targets = []
    for goal in goals:
        targets.append(goal_target(goal, obstacles, goals))
    targets = np.asarray(targets)
    return targets

# Outputs a valid target in a goal as a list object
def goal_target(goal, obstacles, goals):
    polyV = goal.getVrep()
    center = np.mean(polyV, axis=0)
    

    if in_collision(center, obstacles, 0):
        found = False
        while not found:
            target = random_point(goal)
            if not in_collision(target, obstacles, 0):
                found = True
    else:
        target = center
    return target

def decToBin(n, dim):
    temp = [int(x) for x in list('{0:0b}'.format(n))]
    for i in range(len(temp) - 1, dim - 1):
        temp.insert(0, 0)
    return temp

# Returns a random point within a polytope as a list object
def random_point(poly):
    point = []
    polyv = poly.getVrep
    bbox = poly.get_bbox()
    n = len(bbox) / 2
    for i in range(0, n):
        point.append(bbox[2*i] + abs(random.random()*(bbox[2*i + 1] - bbox[2*i])))
    return point