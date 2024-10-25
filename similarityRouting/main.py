
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
    
    #defined template
    path   = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    path_obs_o   = rtGraph.setNetTemplate(centerPts[0], l-1, 2)  #tempalte   Z
    path_obs=[]
    if(len(centerPts) >1):
        path_obs     = rtGraph.setNetTemplate(centerPts[1], 0, 2)  #show z template obstacle
        

    #Set obstacle and show
    obstacles.extend(path_obs_o)
    obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles)  
    rtGraph.drawShapelyPolygon(obstacles, w, h, "black")
    plt.show()

    mkt = mikami.MikamiTauchi(rtGraph)
 
    mkt.findPath(s_start, s_goal)

if __name__ == '__main__':
    s_start = (4, 15, 0)
    s_goal  = (40, 5, 0)
    tpZ     = [(4,15, 0),(28,15, 0),(28,5, 0), (40,5, 0)]
    tpZ1    = [(4,10, 0),(25,10, 0),(25,27, 0), (45,27, 0)]
    # tpZ1    = [(0,10, 0),(25,10, 0),(25,27, 0), (49,27, 0)]
    ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    testMikamiTauchi([tpZ, tpZ1], [], s_start, s_goal, True)    