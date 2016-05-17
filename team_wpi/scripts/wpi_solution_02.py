#!/usr/bin/env python
from __future__ import print_function
import time
import numpy as np
import numpy.linalg as la
from control import lqr

import roslib; roslib.load_manifest('team_wpi')
import rospy
from std_msgs.msg import Header
from integrator_chains_msgs.msg import VectorStamped, Vector, ProblemInstanceJSON
from integrator_chains_msgs.srv import DMMode, DMModeRequest

from fmrb import integrator_chains

### CUSTOM IMPORTS
import heapq
import math

from other_toolbox import *
from prm_toolbox import *
from tsp_toolbox import *

class StateFeedback:
    def __init__(self, intopic, outtopic, K=None, realizable=True):
        self.error = None
        self.intopic = rospy.Publisher(intopic, VectorStamped, queue_size=1)
        while self.intopic.get_num_connections() == 0 and not rospy.is_shutdown():
            time.sleep(0.5)
        if realizable:
            self.outtopic = rospy.Subscriber(outtopic, VectorStamped, self.read_state)
        else:
            self.outtopic = rospy.Subscriber(outtopic, VectorStamped, self.declare_unrealizable)
        self.K = K
        self.trial_ended = False

    def read_state(self, vs):
        if len(vs.v.point) == 0:
            self.trial_ended = True
            return
        self.error = np.asarray(vs.v.point) - self.target_state
        self.intopic.publish(VectorStamped(header=Header(stamp=rospy.Time.now() ), v=Vector(-np.dot(self.K, self.error))))

    def declare_unrealizable(self, vs):
        self.intopic.publish(VectorStamped(header=Header(stamp=rospy.Time.now()), v=Vector([])))
        if len(vs.v.point) == 0:
            self.trial_ended = True
            return

    def unregister(self):
        self.outtopic.unregister()
        self.intopic.unregister()

class LQRController(StateFeedback):
    def __init__(self, intopic, outtopic,
                 A, K, target=None, Q=None, R=None, realizable=True):
        self.target_state = np.zeros(A.shape[0])

        if target is not None:
            self.target_state[:target.shape[0]] = target

        StateFeedback.__init__(self, intopic, outtopic, K, realizable=realizable)

class InstanceMonitor:
    def __init__(self, problemJSON_topic='dynamaestro/probleminstance_JSON'):
        self.instanceJSONsub = rospy.Subscriber(problemJSON_topic,
                                                ProblemInstanceJSON,
                                                self.get_newinstance, queue_size=1)
        self.last_updated = None
        self.prob = None
        self.busy = False

    def get_newinstance(self, pinst):
        self.busy = True
        self.prob = integrator_chains.Problem.loadJSON(pinst.problemjson)
        self.last_updated = pinst.stamp
        self.busy = False

def main(imon):
    start_time = rospy.Time.now()
    dmmode = rospy.ServiceProxy("dynamaestro/mode", DMMode)

    while not dmmode(DMModeRequest.READY):
        time.sleep(0.5)
    assert dmmode(DMModeRequest.START)

    while (not rospy.is_shutdown()
           and (imon.last_updated is None or start_time > imon.last_updated)):
        time.sleep(0.5)

    # Load trial configuration
    n = imon.prob.output_dim
    m = imon.prob.number_integrators
    init = imon.prob.Xinit
    bounds = imon.prob.Y.K[0]
    goals = imon.prob.goals
    obstacles = imon.prob.obstacles

    print("-------------------")
    print("Number of Dimensions: " + str(n))
    print("Number of Integrators: " + str(m))
    print("Number of Goals: " + str(len(goals)))
    print("Number of Obstacles: " + str(len(obstacles)))

    # Get start position
    start = init[0:n]
    start = tuple(start)

    # Number of waypoints in between goal regions
    samples = 10

    # LQR controller
    A = np.diag(np.ones((m - 1)*n), k=n)
    B = np.zeros((m*n, n))
    B[(m - 1)*n:,:] = np.eye(n)
    Q = np.eye(m*n)
    R = np.eye(n)
    K, S, E = lqr(A,B,Q,R)

    # Check if initial position is in any obstacles,
    #   takes the start position as a tuple and the obstacles as a list of polytopes
    if in_collision(start, obstacles, 0):
        print('Robot spawned inside obstacle!')
        waypoints = np.asarray([start])
        realizable = False

    elif not goalsRealizable(goals, obstacles):
        # Check if any goals are completely encased by obstacles
        print('A goal is completely encased by obstacles!')
        waypoints = np.asarray([start])
        realizable = False

    else:
        # Sequence targets within each goal while selecting valid targets within each goal
        #  Takes lists of polytopes and gives a numpy array
        targets = goal_targets(goals, obstacles)

        # TSP gives correlated and optimized goals and targets
        #  Takes the start position as a tuple and the goals as a numpy array of polytopes
        #  and the targets as a numpy array of numpy position arrays
        goals = np.asarray(goals)
        (tsp_goals, tsp_targets) = tsp(start, goals, targets)

        #  Planner
        #  If any planner point passes through another goal, also remove it from the path planning.
        #  If planner fails, it SHOULD mean that no complete path exists.
        waypoints = prm_waypoint(start, samples, bounds, tsp_targets, tsp_goals, obstacles)
        if len(waypoints) == 0:
            print("Path not found.")
            waypoints = np.asarray([start])
            realizable = False
        else:
            way_to_start = prm_waypoint(tuple(tsp_targets[len(tsp_targets)-1]), samples, bounds, [tsp_targets[0]], [tsp_goals[0]], obstacles)
            if len(way_to_start) == 0:
                print("Path not found.")
                waypoints = np.asarray([start])
                realizable = False
            else:
                print('This trial is realizable.')
                realizable = True
                for i in range(0, len(way_to_start)):
                    waypoints.append(way_to_start[i])

                waypoints = np.asarray(waypoints)

    # Run controller
    current = 0
    while (not rospy.is_shutdown() ):
        lqrc = LQRController("input", "state", A, K, waypoints[current], Q, R, realizable=realizable)
        while (not rospy.is_shutdown() and not lqrc.trial_ended and ((lqrc.error is None) or (la.norm(lqrc.error) > 0.4))): # 0.01
            time.sleep(0.05)
        lqrc.unregister()
        if lqrc.trial_ended:
            return
        print('Reached Waypoint ' + str(current) + '.')
        current += 1
        if current >= len(waypoints):
            current = samples + 1

if __name__ == "__main__":
    rospy.init_node("wpi_solution_02", anonymous=True)
    imon = InstanceMonitor()
    number_trials = rospy.get_param('/number_trials', None)
    trial_counter = 0
    while (number_trials is None) or (trial_counter < number_trials):
        main(imon)
        if number_trials is not None:
            trial_counter += 1