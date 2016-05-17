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

# Implement the Planner algorithm to compute a path for the robot
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
        if (4 < path_plan_time):
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

def prm_waypoint(start, samples, bounds, targets, goals, obstacles): # With waypoints
    first = True
    goal_counter = 0
    waypoints = []
    n = len(start)

    for goal in goals:
        planning_start_time = time.time()

        path_length = 0

        if first:
            first = False
            cur_path = start # required by recursive random walk
            while (path_length == 0):
                new_path = goto(cur_path, goal_counter, bounds, targets, goals, obstacles)

                path_length = len(new_path)
                if path_length == 0:     	# begin random path routine
                    if len(start) ==  1: 	# random path impossible for dimension 1
                        waypoints = []
                        return waypoints
                    else:
                        dist = np.inf 		# init infinity
                        while(2*n < dist): 	# bound on how far should new random point be
                            new_point = random_env_point(bounds, obstacles, n) # get new
                            dist = distance(new_point, cur_path)
                        random_path = goto_random(cur_path, bounds, new_point, obstacles)
                        cur_path = random_path[len(random_path) - 1]
                        #print(cur_path)
                
                time_elapsed = time.time() - planning_start_time
                if (time_elapsed > 20*n):
                    waypoints = []
                    return waypoints

        else:
            while(path_length == 0) :
                new_path = goto(cur_path, goal_counter, bounds, targets, goals, obstacles)

                path_length = len(new_path)
                if path_length == 0:     	# begin random path routine
                    if len(start) ==  1: 	# random path impossible for dimension 1
                        waypoints = []
                        return waypoints
                    else:
                        dist = np.inf 		# init infinity
                        while(10 < dist): 	# bound on how far should new random point be
                            new_point = random_env_point(bounds, obstacles, n) # get new
                            dist = distance(new_point, cur_path)
                        random_path = goto_random(cur_path, bounds, new_point, obstacles)
                        cur_path = random_path[len(random_path) - 1]

                time_elapsed = time.time() - planning_start_time
                if (time_elapsed > 20*n):
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

def random_env_point(bounds, obstacles, n):
    found = False
    while not found:
        point = []
        for i in range(0, n):   # create new point in given bounds
            point.append(-bounds + abs(random.random()*(2*bounds))) # append each dimension
        found = not in_collision(point , obstacles, 0)
    point = list(point)    # return only if point is not in collison
    return point

def goto_random(start, bounds, random_point, obstacles):
    frontier = PriorityQueue()
    frontier.put(start, 0)

    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    path = []
    n = len(start)
    obstacle_threshold = 0.3

    path_search_state_time = time.time()

    while not frontier.empty():
        current_state = frontier.get()
        if distance(current_state, random_point) < 0.1:
            break

        for next in neighbors(current_state, bounds):
            new_cost = cost_so_far[current_state] + cost(current_state, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = heuristic(random_point, next)
                if not in_collision(next, obstacles, obstacle_threshold):
                    frontier.put(next, priority)
                came_from[next] = current_state

        if 0.25 < ( time.time() - path_search_state_time):
            dist = np.inf
            while(n*2 < dist): # bound on how far should new random point be
                new_point = random_env_point(bounds, obstacles, n) # get new
                dist = distance(new_point, start)
            path = goto_random(start, bounds, new_point, obstacles)
            return path

    print("Found Random walk ")
    if distance(current_state, random_point) < 0.1:
        path = [current_state]
        while current_state != start:
           current_state = came_from[current_state]
           path.append(current_state)
        path.reverse()
    return path