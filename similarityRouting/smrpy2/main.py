
from __future__ import absolute_import
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import routingGraph
import mikami
import matplotlib.pyplot as plt

def testMikamiTauchi(centerPts, obstacles, s_start, s_goal, multilayer= False):
    u""" mikami-tauchi algorithm test
    Args:
        centerPts list[tuple(0,0,0), ...]:            The center line of template
        obstacles list[[(0,0,0),(0,0,0)], poly2,...]: The obstacle for current layout
        s_start tuple(0,1,1):   The start point for target net
        s_goal  tuple(0,2,12):  The goal point for target net
        multilayer (bool, optional): Whether support multi-layer routing. Defaults to False.
    """
    #Initialized the  routing graph
    w, h, l,r = 50, 50, 3,1
    rtGraph = routingGraph.Grid3D(w, h, l, r)

    #Initialize layout obstacle to routing graph
    path_obs=[]
    if(len(centerPts) >1):
        for i in range(1, len(centerPts)):
            path_obs     = rtGraph.setNetTemplate(centerPts[i], 0, 2)  #show z template obstacle
            obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles) 
    #show design
    fig, ax = plt.subplots(figsize=(w, h)) 
    ax.set_xlim(0, w + 0.1)
    ax.set_ylim(0, h + 0.1)
    rtGraph.drawShapelyPolygon(fig, ax, obstacles,  u"black", 1)
    templateShow   = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    rtGraph.drawShapelyPolygon(fig, ax, templateShow,  u"yellow", 0.4)

    #Find current net's path with template
    mkt = mikami.MikamiTauchi(rtGraph, multilayer)
    mkt.findPathWithTemplate(s_start, s_goal, centerPts[0])

    print u"Routing finish!\n"
    
if __name__ == u'__main__':
    # Z shape
    s_start = (4, 15, 0)
    s_goal  = (40, 5, 0)
    tpZ     = [(4,15, 0),(28,15, 0),(28,5, 0), (40,5, 0)]
    tpZ1    = [(4, 10, 0),(25,10, 0),(25,27, 0), (45,27, 0)]
    # tpZ1    = [(0,10, 0),(25,10, 0),(25,27, 0), (49,27, 0)]
    # ob      = [(45,19, 0), (43,19, 0), (43,27, 0), (45,27, 0)]
    testMikamiTauchi([tpZ, tpZ1], [], s_start, s_goal, True)  

    # U shape
    s_start = (5, 5, 0)
    s_goal  = (20, 5, 0)
    tpZ     = [(5,5, 0),(5,40, 0),(20,40, 0), (20, 5, 0)]
    ob      = [[20, 15,0], [30, 15,0], [30, 20,0], [20, 20,0]]  
    # testMikamiTauchi([tpZ], [], s_start, s_goal, True)
    # testMikamiTauchi([tpZ], [ob], s_start, s_goal, True)

    # snake shape
    s_start = (30,10, 0)
    s_goal  = (45, 45, 0)
    tpZ     = [(30,10,0),(30,45,0),(40,45,0), (40,10,0),(45, 10,0), (45, 45,0)]
    # ob      = [[20, 15,0], [27, 15,0], [27, 20,0], [20, 20,0]]
    ob1      = [[37, 15,0], [41, 15,0], [41, 20,0], [37, 20,0]]
    # ob2      = [[37, 15,0], [46, 15,0], [46, 20,0], [37, 20,0]]
    # testMikamiTauchi([tpZ], [ob1], s_start, s_goal, True)

    # O shape
    s_start = (20, 20, 0)
    s_goal  = (22, 20, 0)
    tpZ     = [(20,20,0), (10, 20,0), (10, 5,0), (30, 5,0), (30, 20,0), (22, 20,0)]
    ob      = [[20, 3,0], [22, 3,0], [22, 10,0], [20, 10,0]]
    ob1     = [[27, 10,0], [33, 10,0], [33, 15,0], [27, 15,0]]
    # testMikamiTauchi([tpZ], [ob, ob1], s_start, s_goal, True)
    
    s_start = (6, 24, 0)
    s_goal  = (4, 24, 0)
    tpZ     = [(6, 24, 0), (17,24,0), (17, 4, 0), (4, 4, 0),(4,24,0)]
    tpZ1    = [(6,10,0),(10,10,0),(10,16,0),(6,16,0)]
    tpZ2    = [(6,7,0), (14, 7, 0),(14, 20, 0), (6, 20,0)]
    # ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    # testMikamiTauchi([tpZ, tpZ1, tpZ2], [], s_start, s_goal, True)

    # cross lines
    s_start = (20, 30, 0)
    s_goal  = (40, 25, 0)
    tpZ1     = [(20,30, 0),(20,25, 0),(25,25, 0), (25,15, 0), (35,15, 0), (35,25, 0), (40,25, 0)]
    tpZ2    = [(24,40, 0),(24,30, 0),(30, 30, 0), (30,20, 0)]
    # tpZ1    = [(0,10, 0),(25,10, 0),(25,27, 0), (49,27, 0)]
    ob      = [(20, 15, 0), (30, 15,0), (30, 20,0), (20, 20,0)]
    testMikamiTauchi([tpZ1, tpZ2], [], s_start, s_goal, True)


    s_start = (10, 20, 0)
    s_goal  = (35, 34, 0)
    tp      = [(10, 20, 0),(29,20, 0),  (29,34, 0),(35, 34, 0)]
    # tp      = [(10, 20, 0),(29,20, 0),(29,20, 1), (29,34, 1), (29,34, 0),(35, 34, 0)]
    testMikamiTauchi([tp, tpZ1, tpZ2], [], s_start, s_goal, True)
