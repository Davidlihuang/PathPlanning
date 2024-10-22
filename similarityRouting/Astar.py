"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import Polygon as MatplotlibPolygon
from shapely.geometry import LineString
from shapely.geometry.polygon import Polygon
from collections import deque

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import plotting,env
import routingGraph

class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, rtGraph, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.rtGraph  = rtGraph
        self.detourCost = 0
        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come
        self.smrWeight  = 3
        self.cstWeight  = 1
    def isOccupied(self, pt):
        print("pt", pt)
        return self.rtGraph.is_occupied(pt)
    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """
        if( self.isOccupied(self.s_start) or self.isOccupied(self.s_goal)):
            print("start or goal is occupied!")
            return  self.extract_path(self.PARENT), self.CLOSED
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)
                if(False == self.rtGraph.isStraitLine(self.PARENT[s], s, s_n)):
                    new_cost += self.detourCost
                
                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED
 
    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """
        diagonal = True
        if self.heuristic_type =="manhattan":
            diagonal=False
        return self.rtGraph.get_neighbors(s[0], s[1], s[2], diagonal)
   
    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """
        heuristic_type = self.heuristic_type  # heuristic type "euclidean"  "manhattan"
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1]) + abs(goal[2] - s[2])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1],  (goal[2] - s[2]))
    
    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf
        m = abs(s_goal[0] - s_start[0]) + abs( s_goal[1] - s_start[1])
        return m
    
    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """
        
        self.cstWeight = 1
        self.smrWeight = 4
        smrCost = max(float(self.rtGraph.s0 - self.rtGraph.getSmrValue(s)),0)
        norm    = max(float(self.g[s] + self.heuristic(s) + smrCost),1)
         
        f = self.cstWeight*(self.g[s]/norm + self.heuristic(s)/norm) +   self.smrWeight*(smrCost/norm)
         
        f1  = self.g[s]  + self.heuristic(s)
        return f
    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """
        if self.rtGraph.is_occupied(s_start) or self.rtGraph.is_occupied(s_end):
            return True
            
        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if self.rtGraph.is_occupied(s1) or self.rtGraph.is_occupied(s2):
                return True
        return False

    

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """
        if len(PARENT) ==0 :
            return []
        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    
def runTemplate(centerPts, obstacles, s_start, s_goal):
    #initialized the  routing graph
    w, h, l,r = 50, 50, 3,1
    rtGraph = routingGraph.Grid3D(w, h, l, r)
    
    #defined template
    path   = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    path_obs=[]
    if(len(centerPts) >1):
        path_obs  = rtGraph.setNetTemplate(centerPts[1], 0, 2)  #show z template obstacle
    
    #initialized similarity graph
    rtGraph.calSmrWithBFS(path)
    rtGraph.drawSmrMap()

    #Set obstacle and show
    obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles)  
    rtGraph.drawShapelyPolygon(obstacles, w, h)
    plt.show()

    """AStar"""
    astar         = AStar(rtGraph, s_start, s_goal, "manhattan")
    plot          = plotting.Plotting(rtGraph, s_start, s_goal)
    path, visited = astar.searching()
    plot.animation(path, visited, "A*")  # animation
    path          = rtGraph.simplePointSet(path)
    print("Path:", path)

          
if __name__ == '__main__':

    #template Z
    s_start = (4, 4, 0)
    s_goal  = (45, 27, 0)
    tpZ     = [(4,4),(25,4),(25,27), (45,27)]
    ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    # runTemplate([tpZ], [ob], s_start, s_goal)

    # template U
    s_start = (5, 5, 0)
    s_goal  = (20, 5, 0)
    tpZ     = [(5,5),(5,40),(20,40), (20, 5)]
    ob      = [[20, 15,0], [30, 15,0], [30, 20,0], [20, 20,0]]
    # runTemplate([tpZ], [ob], s_start, s_goal)
    
    # snake shape
    s_start = (30,10, 0)
    s_goal  = (45, 45, 0)
    tpZ     = [(30,10),(30,45),(40,45), (40,10),(45, 10), (45, 45)]
    # ob      = [[20, 15,0], [27, 15,0], [27, 20,0], [20, 20,0]]
    # ob1      = [[37, 15,0], [41, 15,0], [41, 20,0], [37, 20,0]]
    ob2      = [[37, 15,0], [46, 15,0], [46, 20,0], [37, 20,0]]
    # runTemplate([tpZ], [ob2], s_start, s_goal)


    #O shape
    s_start = (20, 20, 0)
    s_goal  = (22, 20, 0)
    tpZ     = [(20,20), (10, 20), (10, 5), (30, 5), (30, 20), (22, 20)]
    ob      = [[20, 3,0], [22, 3,0], [22, 10,0], [20, 10,0]]
    ob1     = [[27, 10,0], [33, 10,0], [33, 15,0], [27, 15,0]]
    # runTemplate([tpZ], [ob, ob1], s_start, s_goal)

    #cross talk
    s_start = (4, 15, 0)
    s_goal  = (30, 5, 0)
    tpZ     = [(4,15),(28,15),(28,5), (40,5)]
    tpZ1    = [(6,10),(25,10),(25,27), (45,27)]
    ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    runTemplate([tpZ, tpZ1], [], s_start, s_goal)