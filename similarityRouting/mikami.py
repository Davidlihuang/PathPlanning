
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
from routingGraph import Grid3D
import rtree
from enum import Enum

## data structure
class TopologyEnum(Enum):
    LEFT       = 0
    RIGHT      = 1
    UP         = 2
    DOWN       = 3
    FRONT      = 4  # VIA DRILL UP
    BACK       = 5  # VIA DRILL DOWN
    ANY        = -1

class AxisEnum(Enum):
    AXIS_X     = 0
    AXIS_Y     = 1
    AXIS_Z     = 2
    AXIS_OTHER = -1
    
class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Line: 
    def __init__(self, st:Point, ed:Point):
        self.id:int        = -1
        self.origin:Point                 #current line origin point
        self.start:Point   = st           #start point
        self.end:Point     = ed           #end   point
        self.parent:dict   = dict()       #{Line:Line}
        self.pointSet      = [[],[],[]]   #[[x:Point], [y:Point], [z:Point]] 
        self.axisDirect:AxisEnum     = AxisEnum.AXIS_OTHER
        self.lineDirect:TopologyEnum = TopologyEnum.ANY
    def setId(self, id):
        self.id = id
    def setOrigin(self, origin:Point):
        self.origin   = origin  
    def setStart(self, pt:Point):
        self.start = pt
    def setEnd(self, pt:Point):
        self.end = pt    
    def setParent(self, line):
        self.parent[line] = line
    def setAxisType(self, axis:AxisEnum):
        self.axisDirect = axis
    def setLineDirect(self, topo: TopologyEnum):
        self.lineDirect = topo
    def setAxisPoints(self, points:list, axis: AxisEnum):
        for p in points:
            self.addPoint(p, axis)
    def addPoint(self, pt: Point, axis: AxisEnum):
        self.pointSet[axis.value].append(pt)        

class LineContainer:
    def __init__(self):
        self.lines                = [[],[],[]]     #[[x:Line], [y:Line], [z:Line]] 
        self.container            = [rtree.index.Index(), rtree.index.Index(),rtree.index.Index()]
        self.lineDict             = dict()
    def __calSize(self):
        len = 0
        for lineList in self.lines:
            len += len(lineList)
        return len
    def queryIntersect(self, line:Line):
        itsList = []
        minPoint = Point(min(line.start.x, line.end.x), min(line.start.y, line.end.y), min(line.start.z, line.end.z))
        maxPoint = Point(max(line.start.x, line.end.x), max(line.start.y, line.end.y), max(line.start.z, line.end.z))

        #@todo
        for item in self.container:
            intersecting_segments = item.intersection((minPoint.x, minPoint.y, maxPoint.x, maxPoint.y))
            for sid in intersecting_segments:
                curLine = self.lineDict[sid]
                tmpLine = LineString([(curLine.start.x, curLine.start.y, curLine.start.z), (curLine.end.x, curLine.end.y, curLine.end.z)])
                spLine  = LineString([(line.start.x, line.start.y, line.start.z), (line.end.x, line.end.y, line.end.z)])
                if spLine.intersection(tmpLine) or spLine.equals(tmpLine):
                    itsList.append(curLine)
        return itsList
    
    def getLines(self, axis: AxisEnum):
        if (axis == AxisEnum.AXIS_X):
            return self.lines[0]
        elif (axis == AxisEnum.AXIS_Y):
            return self.lines[1]
        elif (axis == AxisEnum.AXIS_Z):
            return self.lines[2]
        else:
            return []
        
    def setLines(self, lines, axis:AxisEnum):
        for line in lines:
            self.addLine(line, axis)

    def addLine(self, line:Line, axis: AxisEnum):
        if(axis.value < AxisEnum.AXIS_X.value or axis.value > AxisEnum.AXIS_Z.value):
            print("Error: invalid index:{}".format(axis))
            return

        self.lines[axis.value].append(line)
        self.lineDict[line.id] = line
        
        #add to RTree
        minPoint = Point(min(line.start.x, line.end.x), min(line.start.y, line.end.y), min(line.start.z, line.end.z))
        maxPoint = Point(max(line.start.x, line.end.x), max(line.start.y, line.end.y), max(line.start.z, line.end.z))
        self.container[axis.value].insert(int(line.id), (minPoint.x, minPoint.y, maxPoint.x, maxPoint.y), obj=line)

        
class MikamiTauchi:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, rtGraph:Grid3D):
        self.rtGraph:Grid3D   = rtGraph
        self.PARENT:dict    = dict()
        self.allLines:dict  = dict()
        self.startSet:LineContainer  = LineContainer()
        self.targetSet:LineContainer = LineContainer()
        self.smrWeight      = 3
        self.cstWeight      = 1
    def _isValid(self, pt:Point):
        return self.rtGraph.isValid((pt.x, pt.y, pt.z))
    def _isOccupied(self, pt:Point):
        return self.rtGraph.is_occupied((pt.x, pt.y, pt.z))
    def findPath(self, s, e):
        start = Point(s[0], s[1], s[2])
        goal  = Point(e[0], e[1], e[2])
        # invalid
        if self.rtGraph.is_occupied((start.x, start.y, start.z)):
            print("Start is occupied!")
            return []
        if self.rtGraph.is_occupied((goal.x, goal.y, goal.z)):
            print("Target is occupied")
            return []
        
        #create s/t line
        _, lineX_s = self.createLine(start, AxisEnum.AXIS_X)
        self.startSet.addLine(lineX_s,   AxisEnum.AXIS_X)
        _, lineY_s = self.createLine(start, AxisEnum.AXIS_Y)
        self.startSet.addLine(lineY_s,   AxisEnum.AXIS_Y)
        _, lineZ_s = self.createLine(start, AxisEnum.AXIS_Z)
        self.startSet.addLine(lineZ_s,   AxisEnum.AXIS_Z)

        _, lineX_t = self.createLine(goal, AxisEnum.AXIS_X)
        self.targetSet.addLine(lineX_t, AxisEnum.AXIS_X)
        _, lineY_t = self.createLine(goal, AxisEnum.AXIS_Y)
        self.targetSet.addLine(lineY_t, AxisEnum.AXIS_Y)
        _, lineZ_t = self.createLine(goal, AxisEnum.AXIS_Z)
        self.targetSet.addLine(lineZ_t, AxisEnum.AXIS_Z)
        
        lines = self.targetSet.queryIntersect(lineX_t)
        print(lines)
    
    def findPathWithTemplate(self, start:Point, goal:Point, tp:list):
        pass

    def createLine(self, coord: Point, axis:AxisEnum, tpDir:TopologyEnum=TopologyEnum.ANY):
        line = Line(coord, coord)
        if( self._isValid(coord) == False):
            return (False, line)
        
        line.setOrigin(coord)
        line.setAxisType(axis)
        line.setLineDirect(tpDir)
 
        if(AxisEnum.AXIS_X == axis):
            points = set()
            for x in range(coord.x, self.rtGraph.w):
                tmpPoint = Point(x, coord.y, coord.z)
                if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                    break
                points.add(tmpPoint)
            for x in range(coord.x, -1, -1):
                tmpPoint = Point(x, coord.y, coord.z)
                if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                    break
                points.add(tmpPoint)

            points_list = list(points)
            points_list.sort(key=lambda point: point.x)
            bigger =  max([pt.x for pt in points_list])
            small  =  min([pt.x for pt in points_list])
            line.setStart(Point(small, coord.y, coord.z))
            line.setEnd(Point(bigger, coord.y, coord.z))
            for pt in points_list:
                line.addPoint(pt, axis)
            
        elif AxisEnum.AXIS_Y == axis:
            points = set()
            for y in range(coord.y, self.rtGraph.h):
                tmpPoint = Point(coord.x, y, coord.z)
                if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                    break
                points.add(tmpPoint)
            for y in range(coord.y, -1, -1):
                tmpPoint = Point(coord.x, y, coord.z)
                if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                    break
                points.add(tmpPoint)
            points_list = list(points)
            points_list.sort(key=lambda point: point.y)
            bigger =  max([pt.y for pt in points_list])
            small =  min([pt.y for pt in points_list])
            line.setStart(Point(coord.x, small, coord.z))
            line.setEnd(Point(coord.x, bigger, coord.z))
            for pt in points_list:
                line.addPoint(pt, axis)
        elif AxisEnum.AXIS_Z == axis:
            points = set()
            for z in range(coord.z, self.rtGraph.l):
                tmpPoint = Point(coord.x, coord.y, z)
                if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                    break
                points.add(tmpPoint)
            for z in range(coord.z, -1, -1):
                tmpPoint = Point(coord.x, coord.y, z)
                if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                    break
                points.add(tmpPoint)
            points_list = list(points)
            points_list.sort(key=lambda point: point.z)
            bigger =  max([pt.z for pt in points_list])
            small  =  min([pt.z for pt in points_list])
            line.setStart(Point(coord.x, coord.y, small))
            line.setEnd(Point(coord.x, coord.y, bigger))
            for pt in points_list:
                line.addPoint(pt, axis)
        else:
            pass

        id = len(self.allLines)
        line.setId(id)
        self.allLines[id] = line
        return (True, line)
        
    def expandLine(self, line:Line, tpDir:TopologyEnum=TopologyEnum.ANY):
        tmpLine = []

        return tmpLine