"""
RRT_2D
@author: huiming zhou
"""

from genericpath import exists
import os
import sys
import math
import numpy as np

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../Sampling_based_Planning/")

import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        # self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y, 2, 0)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y, 2, 0))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

def path_planner():

    path = np.array([[1,1,2,0]])
    interp_points = np.array([])
    end = (1,1,2,0)
    pos_1 = (7.16978773, 5.77663535, 2, 0.622398769648)
    pos_2 = (2.87358722, 0.87698614, 2, -1.33810462117)
    pos_3 = (1.99449893, 6.60133113, 2, -3.34442834176)
    pos_4 = (8.39216226, 4.51662851, 2, -0.42706930097)

    goal_position = np.array([end, pos_1, pos_2, pos_3, pos_4, end])

    for i in range(5):
        x_start = goal_position[i]  # Starting node
        # print(x_start)
        x_goal = goal_position[i+1]  # Goal node
        # print(x_goal)
        rrt = Rrt(x_start, x_goal, 0.75, 0.05, 10000)
        curr_path = np.flipud(np.array(rrt.planning()))
        interp_points = np.append(interp_points, np.shape(curr_path)[0])
        # print(np.size(curr_path))
        path = np.append(path, curr_path, axis=0)

    pose_01 = np.linspace(0, pos_1[3], interp_points[0])    
    pose_12 = np.linspace(pos_1[3], pos_2[3], interp_points[1])    
    pose_23 = np.linspace(pos_2[3], pos_3[3], interp_points[2])    
    pose_34 = np.linspace(pos_3[3], pos_4[3], interp_points[3])    
    pose_40 = np.linspace(pos_4[3], 0, interp_points[4]+1)    

    pose_total = np.append(pose_01, pose_12)
    pose_total = np.append(pose_total, pose_23)
    pose_total = np.append(pose_total, pose_34)
    pose_total = np.append(pose_total, pose_40)

    print(sum(interp_points))
    print(np.shape(pose_total))
    print(np.shape(path))
    for i in range(np.shape(path)[0]):
        path[i][3] = pose_total[i]+1.57

    print(path)
    return path