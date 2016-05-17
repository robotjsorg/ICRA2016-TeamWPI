#!/usr/bin/env python
from __future__ import print_function
import time
import numpy as np
import numpy.linalg as la

import roslib; roslib.load_manifest('team_wpi')
import rospy
from std_msgs.msg import Header

import heapq
import math

from other_toolbox import *

# Import standard priority queue definitions
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

# Neighbors in 4 connected space: 2n neighbours
def neighbors(home, bounds):
    nbr = []
    step = 0.01
    in_bounds = True
    for j in range(0, len(home)):
        if -bounds >= home[j] >= bounds:
            in_bounds = False
            break

    if in_bounds == True:
        for j in range(0, len(home)):
            n1 = list(home)
            n1[j] = round(home[j] + step, 2)
            n2 = list(home)
            n2[j] = round(home[j] - step, 2)
            nbr.append(tuple(n1))
            nbr.append(tuple(n2))
        nbr = tuple(nbr)
    return nbr

def heuristic(pos1, pos2):
    heur = distance(pos1, pos2)
    return heur

def cost(pos1, pos2):
    cost = distance(pos1, pos2) 
    return cost

# Implement the A* algorithm to compute a path for the robot
def goto(start, index, bounds, targets, goals, obstacles):
    frontier = PriorityQueue()
    frontier.put(start, 0)

    n = len(start)

    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    path = []

    obstacle_threshold = 0.3
    goal_threshold = -0.2

    target = targets[index]

    path_search_state_time = time.time()

    while not frontier.empty():
        current_state = frontier.get()
        if in_goal(current_state, index, goals, goal_threshold):
            break

        for next in neighbors(current_state, bounds):
            new_cost = cost_so_far[current_state] + cost(current_state, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = heuristic(target, next)
                if not in_collision(next, obstacles, obstacle_threshold):
                    frontier.put(next, priority)
                came_from[next] = current_state
        path_plan_time = time.time() - path_search_state_time
        if (2*n < path_plan_time):
            path = []
            return path
    #print('Path planning time: ' + str(time.time() - path_search_state_time))
    
    if in_goal(current_state, index, goals, goal_threshold):
        path = [current_state]
        while current_state != start:
           current_state = came_from[current_state]
           path.append(current_state)
        path.reverse()
    return path

def planner(start, bounds, targets, goals, obstacles): # Without waypoints
    first = True
    goal_counter = 0
    path = []

    for goal in goals:
        if first:
            first = False
            new_path = goto(start, goal_counter, bounds, targets, goals, obstacles)
        else:
            new_path = goto(cur_path, goal_counter, bounds, targets, goals, obstacles)
        path.append(new_path)
        cur_path = new_path[len(path) - 1]
        goal_counter += 1
    return path

def planner_waypoint(start, samples, bounds, targets, goals, obstacles): # With waypoints
    first = True
    goal_counter = 0
    waypoints = []

    for goal in goals:
        if first:
            first = False
            new_path = goto(start, goal_counter, bounds, targets, goals, obstacles)
        else:
            new_path = goto(cur_path, goal_counter, bounds, targets, goals, obstacles)
        path_length = len(new_path)
        if path_length == 0: # and len(start) == 1:
            waypoints = []
            return waypoints


        cur_path = new_path[path_length - 1]

        # Sample waypoints along the path
        sample_size = math.ceil(path_length / samples)
        for sample in range(0, samples):
            waypoints.append(new_path[int(sample * sample_size)])
        waypoints.append(new_path[path_length - 1])
        goal_counter += 1

    return waypoints