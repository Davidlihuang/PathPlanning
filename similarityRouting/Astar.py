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
        self.detourCost = 2
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

    
def runTemplate(centerPts, obstacles, s_start, s_goal, showAnimal= False):
    #initialized the  routing graph
    w, h, l,r = 50, 50, 3,1
    rtGraph = routingGraph.Grid3D(w, h, l, r)
    
    #defined template
    path   = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    path_obs_o   = rtGraph.setNetTemplate(centerPts[0], l-1, 2)  #tempalte   Z
    path_obs=[]
    if(len(centerPts) >1):
        path_obs     = rtGraph.setNetTemplate(centerPts[1], 0, 2)  #show z template obstacle
        
    
    #initialized similarity graph
    # rtGraph.calSmrWithBFS(path)
    rtGraph.calSmrWithBFS(centerPts[0], True)
    rtGraph.drawSmrMap()

    #Set obstacle and show
    obstacles.extend(path_obs_o)
    obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles)  
    rtGraph.drawShapelyPolygon(obstacles, w, h)
    # plt.show()

    """AStar"""
    iter  = 20
    plot          = plotting.Plotting(rtGraph, s_start, s_goal)
    bestPathPoints = []
    bestVisited    = [] 
    bestPath       = []
    oldSmr        = 0.0
    newSmr        = 0.0
    vCnd          = 1.0
    # while(iter>0  and vCnd > 1e-3):
    while(iter>0):
        iter -= 1
        astar         = AStar(rtGraph, s_start, s_goal, "manhattan")
        pathPoints, visited = astar.searching()
        path     = rtGraph.simplePointSet(pathPoints)
        newSmr   = rtGraph.calculateMatchValue(centerPts[0], path)
        disMatch = rtGraph.getDisMatchSegmentRange(centerPts[0], path)
        rtGraph.updateSmrForDisMatchSeg(disMatch, 0.2)
        
        # ignore worst result ignore it, new smr value must bigger than old smr
        if(newSmr < oldSmr and iter>0 ):
            continue

        #update smr map
        bestPathPoints = pathPoints
        bestVisited    = visited
        bestPath       = path
        
        oldSmr = newSmr
        vCnd = newSmr - oldSmr 
        print("smr: {}%\nPath:{}".format(newSmr*100, path))

    print("smr: {}%\nPath:{}".format(newSmr*100, bestPath))
    plot.animation(bestPathPoints, bestVisited, "Best result", enable=showAnimal)  # animation
          
if __name__ == '__main__':

    #template Z
    s_start = (4, 4, 0)
    s_goal  = (45, 27, 0)
    tpZ     = [(4,4,0),(25,4,0),(25,27,0), (45,27,0)]
    ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    # runTemplate([tpZ], [ob], s_start, s_goal)

    # template U
    s_start = (5, 5, 0)
    s_goal  = (20, 5, 0)
    tpZ     = [(5,5, 0),(5,40, 0),(20,40, 0), (20, 5, 0)]
    ob      = [[20, 15,0], [30, 15,0], [30, 20,0], [20, 20,0]]
    # runTemplate([tpZ], [ob], s_start, s_goal)
    
    # snake shape
    s_start = (30,10, 0)
    s_goal  = (45, 45, 0)
    tpZ     = [(30,10,0),(30,45,0),(40,45,0), (40,10,0),(45, 10,0), (45, 45,0)]
    # ob      = [[20, 15,0], [27, 15,0], [27, 20,0], [20, 20,0]]
    ob1      = [[37, 15,0], [41, 15,0], [41, 20,0], [37, 20,0]]
    # ob2      = [[37, 15,0], [46, 15,0], [46, 20,0], [37, 20,0]]
    # runTemplate([tpZ], [ob1], s_start, s_goal)


    #O shape
    s_start = (20, 20, 0)
    s_goal  = (22, 20, 0)
    tpZ     = [(20,20,0), (10, 20,0), (10, 5,0), (30, 5,0), (30, 20,0), (22, 20,0)]
    ob      = [[20, 3,0], [22, 3,0], [22, 10,0], [20, 10,0]]
    ob1     = [[27, 10,0], [33, 10,0], [33, 15,0], [27, 15,0]]
    runTemplate([tpZ], [ob, ob1], s_start, s_goal)

    #cross talk
    s_start = (4, 15, 0)
    s_goal  = (40, 5, 0)
    tpZ     = [(4,15, 0),(28,15, 0),(28,5, 0), (40,5, 0)]
    tpZ1    = [(4,10, 0),(25,10, 0),(25,27, 0), (45,27, 0)]
    # tpZ1    = [(0,10, 0),(25,10, 0),(25,27, 0), (49,27, 0)]
    ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    # runTemplate([tpZ, tpZ1], [], s_start, s_goal)