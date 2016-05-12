#!/usr/bin/env python
from __future__ import print_function
import time
import numpy as np
import numpy.linalg as la

import roslib; roslib.load_manifest('team_wpi_integrator_chains')
import rospy
from std_msgs.msg import Header

import heapq
import math

from other_toolbox import *

# Returns the goals and the targets within them in the optimal order, as numpy arrays
def tsp(start, goals, targets):
    ordered_targets = ordered_target_indices(start, targets)
    tsp_goals = goals[ordered_targets]
    tsp_targets = targets[ordered_targets]
    return tsp_goals, tsp_targets

def ordered_target_indices(start, targets):
    last_target = np.asarray(start)
    ordered_target_indices = []

    while targets.size > 0:
        distances = []
        for target in targets:
            distance_to = distance(last_target, target)
            distances = np.append(distances, distance_to)
        closest_target = np.argmin(distances)
        last_target = targets[closest_target]
        targets = np.delete(targets, (closest_target), axis=0)
        for target_index in np.sort(ordered_target_indices):
            if target_index <= closest_target:
                closest_target += 1
        ordered_target_indices = np.append(ordered_target_indices, closest_target)
    ordered_target_indices = ordered_target_indices.astype(int)
    return ordered_target_indices