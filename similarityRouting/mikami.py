
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
from shapely.geometry import Point as sPoint
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
    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return (self.x == other.x) and (self.y == other.y) and (self.z == other.z)
    def __hash__(self):
        return hash((self.x, self.y, self.z))

class Line: 
    def __init__(self, st:Point, ed:Point):
        self.id:int        = -1
        self.origin:Point                 #current line origin point
        self.start:Point   = st           #start point
        self.end:Point     = ed           #end   point
        self.parent:Line   = None         #{Line:Line}
        self.pointSet      = []
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
        self.parent  = line
    def setAxisType(self, axis:AxisEnum):
        self.axisDirect = axis
    def setLineDirect(self, topo: TopologyEnum):
        self.lineDirect = topo
    def setAxisPoints(self, points:list, axis: AxisEnum):
        for p in points:
            self.addPoint(p, axis)
    def addPoint(self, pt: Point):     
        self.pointSet.append(pt)     
    def getLength(self, manhattan=True):
        len = 0.0
        if (manhattan):
            len = abs(self.end.x - self.start.x) + abs(self.end.y - self.end.y) + abs(self.end.z - self.end.z)
        else:
            len =  math.sqrt( (self.end.x - self.start.x)**2 + (self.end.y - self.end.y)**2 + (self.end.z - self.end.z)**2 )
        return len
    def traceBack(self):
        pts = []
        parent = self
        while (parent != None): 
            pts.append(parent.origin)
            parent = parent.parent
        return pts
            
    def getIntersectionPoint(self, line):
        tmpLine = LineString([(self.start.x, self.start.y, self.start.z), (self.end.x, self.end.y, self.end.z)])
        spLine  = LineString([(line.start.x, line.start.y, line.start.z), (line.end.x, line.end.y, line.end.z)])
        pt = spLine.intersection(tmpLine) 
        if not isinstance(pt, sPoint): 
            return None 
        resPt = Point(pt.x, pt.y, pt.z)
        return resPt
        
    def draw(self,  cr='', lw=1, lsty='-', ):
        crList = ['red', 'blue', 'green', 'yellow','gold']
        if cr =='':
            cr = crList[int(self.start.z)]
        plt.plot([self.start.x, self.end.x],[self.start.y, self.end.y],  linestyle=lsty, linewidth=lw, color=cr)
        for pt in self.pointSet:
            plt.plot(pt.x, pt.y, 'o', markersize=lw+0.5, color='Gray')
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
                if spLine.intersection(tmpLine):
                    itsList.append(curLine)
        return itsList
    def queryEqual(self, line:Line):
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
                if spLine.equals(tmpLine):
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
        
    def setLines(self, lines:list):
        for line in lines:
            self.addLine(line, line.axisDirect)

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

class Path:
    def __init__(self):
        self.segs = []
        self.vias = []
        self.shapes = []
    def addSeg(self, seg:Line):
        self.segs.append(seg)
    def getLength(self):
        tLen = 0.0
        for seg in self.segs:
            tLen += seg.getLength()
        return tLen
    def draw(self, cr='', width = 6):
        for seg in self.segs:
            seg.draw(cr, width)                    
class MikamiTauchi:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, rtGraph:Grid3D):
        self.rtGraph:Grid3D    = rtGraph
        self.allPointsDT:dict    = dict()   #store(state[0,0,0]) : x, y, z 0:1 
        self.allLines:dict     = dict()
        self.startSet:LineContainer  = LineContainer()
        self.targetSet:LineContainer = LineContainer()
        self.smrWeight      = 3
        self.cstWeight      = 1
        self.enMultiLyr     = False
    def _isValid(self, pt:Point):
        return self.rtGraph.isValid((pt.x, pt.y, pt.z))
    def _isOccupied(self, pt:Point):
        return self.rtGraph.is_occupied((pt.x, pt.y, pt.z))

    def getPath(self, line1:Line, line2:Line)->Path:
        path = Path()
        pts  = line1.traceBack()
        pts2 = line2.traceBack()
        pts = pts[::-1]
        itp = line1.getIntersectionPoint(line2)
        if(itp != None):
            pts.append(itp)
        pts.extend(pts2)
        for i in range(len(pts)-1):
            start = pts[i]
            end   = pts[i+1]
            ln  = Line(start, end)
            path.addSeg(ln)
        return  path
    def calIntersection(self, lineSet:list, targetContainer:LineContainer)->Path:
        bestPath:Path = None
        bestLen  = math.inf
        for line1 in lineSet:
            lines = targetContainer.queryIntersect(line1)
            if(len(lines) == 0):
                continue
            else:
                for ll in lines:
                    path = self.getPath(line1, ll)
                    wireLen = path.getLength()
                    if(wireLen < bestLen):
                        bestLen = wireLen
                        bestPath = path
                        path.draw()
        return bestPath
    def findPath(self, s, e):
        #tmp
        visitSource = []
        visitTarget = []

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
        startLines     =  self.createLineAll(start)
        self.startSet.setLines(startLines)
        for l in startLines:
            l.draw('red')
        goalLines   = self.createLineAll(goal)
        self.targetSet.setLines(goalLines)
        for l in goalLines:
            l.draw('green') 
        
        path =  self.calIntersection(startLines, self.targetSet)
        while len(startLines) !=0 and len(goalLines)!=0 and path ==None:
            #expand S
            expandList = []
            for subLine in startLines:
                e = self.expandLine(subLine, self.startSet)
                expandList.extend(e)
            self.startSet.setLines(expandList)
            visitSource.append(expandList)   
            path  = self.calIntersection(expandList, self.targetSet)
            if (path != None):
                print("Find path success from source !")
                break
            startLines =   expandList

            #expand T
            expandListT = []
            for subLine in goalLines:
                e = self.expandLine(subLine, self.targetSet)
                expandListT.extend(e) 
            self.targetSet.setLines(expandListT)
            visitTarget.append(expandListT)
            path  = self.calIntersection(expandListT, self.startSet)
            if (path != None):
                print("Find path success from target !")
                break
            goalLines = expandListT
        
        self.showAlgoResult(visitSource, visitTarget, path)
        plt.show()
        

    def findPathWithTemplate(self, start:Point, goal:Point, tp:list):
        pass
    
    def createLineAll(self, coord: Point):
        lines = []
        _,l1 = self.createLine(coord, AxisEnum.AXIS_X)
        _,l2 = self.createLine(coord, AxisEnum.AXIS_Y)
        
        lines.append(l1)
        lines.append(l2)

        if self.enMultiLyr ==True:
            _,l3 = self.createLine(coord, AxisEnum.AXIS_Z)
            lines.append(l3)

        return lines

    def createLine(self, coord: Point, axis:AxisEnum, tpDir:TopologyEnum=TopologyEnum.ANY):
        line = Line(coord, coord)
        if( self._isValid(coord) == False):
            return (False, line)
        
        line.setOrigin(coord)
        line.addPoint(coord)
        line.setAxisType(axis)
        line.setLineDirect(tpDir)
        line.setParent(None)
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
                line.addPoint(pt) 
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
                line.addPoint(pt)
        elif AxisEnum.AXIS_Z == axis and self.enMultiLyr == True:
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
                line.addPoint(pt)
        else:
            return (False, None)

        id = len(self.allLines)
        line.setId(id)
        self.allLines[id] = line
        return (True, line)
                 
    def expandLine(self, line:Line, lct:LineContainer, tpDir:TopologyEnum=TopologyEnum.ANY):
        #record expanded points, if expand ignore
        lines1  = []
        lines2  = []
        if (line.axisDirect == AxisEnum.AXIS_X):
            for pt in line.pointSet:
                if pt == line.origin:
                    continue
                
                if(self.enMultiLyr == False): 
                    if  False == (pt in self.allPointsDT and self.allPointsDT[pt][AxisEnum.AXIS_Y.value] == 1) :
                        _,lineY = self.createLine(pt,AxisEnum.AXIS_Y, tpDir)
                        lineY.setParent(line)
                        lines1.append(lineY)
                        for point in lineY.pointSet:
                            state = [0,0,0]
                            if point not in self.allPointsDT:
                                state[lineY.axisDirect.value] =  1
                                self.allPointsDT[point] = state
                            else:
                                self.allPointsDT[point][lineY.axisDirect.value] = 1
                else:
                    if  False == (pt in self.allPointsDT and self.allPointsDT[pt][AxisEnum.AXIS_Z.value] == 1) :
                        _,lineZ = self.createLine(pt,AxisEnum.AXIS_Z, tpDir)
                        lineZ.setParent(line)
                        lines2.append(lineZ)
                        for point in lineZ.pointSet:
                            state = [0,0,0]
                            if point not in self.allPointsDT:
                                state[lineZ.axisDirect.value] =  1
                                self.allPointsDT[point] = state
                            else:
                                self.allPointsDT[point][lineZ.axisDirect.value] = 1  
        elif (line.axisDirect == AxisEnum.AXIS_Y):
            for pt in line.pointSet:
                if pt == line.origin:
                    continue
                
                if(self.enMultiLyr == False):
                    if  False == (pt in self.allPointsDT and self.allPointsDT[pt][AxisEnum.AXIS_X.value] == 1) :
                        _,lineX = self.createLine(pt,AxisEnum.AXIS_X, tpDir)
                        lineX.setParent(line)
                        lines1.append(lineX)
                        for point in lineX.pointSet:
                            state = [0,0,0]
                            if point not in self.allPointsDT:
                                state[lineX.axisDirect.value] =  1
                                self.allPointsDT[point] = state
                            else:
                                self.allPointsDT[point][lineX.axisDirect.value] = 1
                else:
                    if  False == (pt in self.allPointsDT and self.allPointsDT[pt][AxisEnum.AXIS_Z.value] == 1) :
                        _,lineZ = self.createLine(pt,AxisEnum.AXIS_Z, tpDir)
                        lineZ.setParent(line)
                        lines2.append(lineZ)
                        for point in lineZ.pointSet:
                            state = [0,0,0]
                            if point not in self.allPointsDT:
                                state[lineZ.axisDirect.value] =  1
                                self.allPointsDT[point] = state
                            else:
                                self.allPointsDT[point][lineZ.axisDirect.value] = 1
        elif (line.axisDirect == AxisEnum.AXIS_Z and self.enMultiLyr == True):
            for pt in line.pointSet:
                if pt == line.origin:
                    continue
                if  False == (pt in self.allPointsDT and self.allPointsDT[pt][AxisEnum.AXIS_X.value] == 1) :
                    _,lineX = self.createLine(pt,AxisEnum.AXIS_X, tpDir)
                    lineX.setParent(line)
                    lines1.append(lineX)
                    for point in lineX.pointSet:
                            state = [0,0,0]
                            if point not in self.allPointsDT:
                                state[lineX.axisDirect.value] =  1
                                self.allPointsDT[point] = state
                            else:
                                self.allPointsDT[point][lineX.axisDirect.value] = 1
                if  False == (pt in self.allPointsDT and self.allPointsDT[pt][AxisEnum.AXIS_Y.value] == 1) :
                    _,lineY = self.createLine(pt,AxisEnum.AXIS_Y, tpDir)
                    lineY.setParent(line)
                    lines2.append(lineY)
                    for point in lineY.pointSet:
                            state = [0,0,0]
                            if point not in self.allPointsDT:
                                state[lineY.axisDirect.value] =  1
                                self.allPointsDT[point] = state
                            else:
                                self.allPointsDT[point][lineY.axisDirect.value] = 1    
        else:
            return []
        lines1.extend(lines2)
        return lines1
    
    def showAlgoResult(self, sourceList, targetList, path):
        for lst in sourceList:
            for l in lst:
                l.draw('gray')
        for lst in targetList:
            for l in lst:
                l.draw('gray')
        
        if path != None:
            path.draw()
        