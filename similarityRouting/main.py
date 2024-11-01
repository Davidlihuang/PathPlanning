
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

import plotting 
import routingGraph
import mikami



def testMikamiTauchi(centerPts, obstacles, s_start, s_goal, showAnimal= False):
    #initialized the  routing graph
    w, h, l,r = 50, 50, 3,1
    rtGraph = routingGraph.Grid3D(w, h, l, r)
    
    #extract topo
    # topo = topologyExtract(centerPts[0])


    #defined template
    path         = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    path_obs_o   = rtGraph.setNetTemplate(centerPts[0], l-1, 2)  #tempalte   Z
    path_obs=[]
    if(len(centerPts) >1):
        path_obs     = rtGraph.setNetTemplate(centerPts[1], 0, 2)  #show z template obstacle
        

    #Set obstacle and show
    obstacles.extend(path_obs_o)
    obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles)  
    rtGraph.drawShapelyPolygon(obstacles, w, h, "black")
    # plt.show()

    mkt = mikami.MikamiTauchi(rtGraph, True)

    mkt.findPathDfs2(s_start, s_goal, centerPts[0])
 
if __name__ == '__main__':
    s_start = (4, 15, 0)
    s_goal  = (40, 5, 0)
    tpZ     = [(4,15, 0),(28,15, 0),(28,5, 0), (40,5, 0)]
    tpZ1    = [(4, 10, 0),(25,10, 0),(25,27, 0), (45,27, 0)]
    # tpZ1    = [(0,10, 0),(25,10, 0),(25,27, 0), (49,27, 0)]
    # ob      = [(45,19, 0), (43,19, 0), (43,27, 0), (45,27, 0)]
    # testMikamiTauchi([tpZ, tpZ1], [], s_start, s_goal, True)  

     # template U
    s_start = (5, 5, 0)
    s_goal  = (20, 5, 0)
    tpZ     = [(5,5, 0),(5,40, 0),(20,40, 0), (20, 5, 0)]
    ob      = [[20, 15,0], [30, 15,0], [30, 20,0], [20, 20,0]]  
    testMikamiTauchi([tpZ], [], s_start, s_goal)

    s_start = (30,10, 0)
    s_goal  = (45, 45, 0)
    tpZ     = [(30,10,0),(30,45,0),(40,45,0), (40,10,0),(45, 10,0), (45, 45,0)]
    # ob      = [[20, 15,0], [27, 15,0], [27, 20,0], [20, 20,0]]
    ob1      = [[37, 15,0], [41, 15,0], [41, 20,0], [37, 20,0]]
    # ob2      = [[37, 15,0], [46, 15,0], [46, 20,0], [37, 20,0]]
    # testMikamiTauchi([tpZ], [ob1], s_start, s_goal, True)

    #O shape
    s_start = (20, 20, 0)
    s_goal  = (22, 20, 0)
    tpZ     = [(20,20,0), (10, 20,0), (10, 5,0), (30, 5,0), (30, 20,0), (22, 20,0)]
    ob      = [[20, 3,0], [22, 3,0], [22, 10,0], [20, 10,0]]
    ob1     = [[27, 10,0], [33, 10,0], [33, 15,0], [27, 15,0]]
    # testMikamiTauchi([tpZ], [], s_start, s_goal, True)


    graph = {"1": ["5", "9"], "5": ["2", "4"], "9": ["8"], "2": ["4"], "4": ["2"], "8": []}
    # print(dfs(graph, "1"))

    line1 = mikami.Line(mikami.Point(1, 1,0), mikami.Point(5,1,0))
    line2 = mikami.Line(mikami.Point(6, 1,0), mikami.Point(10,1,0))
    print("pa: ", line1.isParallel(line2))
    print("pa: ", line1.isCover(line2))