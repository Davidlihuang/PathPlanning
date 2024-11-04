
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
import copy
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
    
    def getCentroid(self):
        x = abs(self.end.x - self.start.x)/2 + min(self.start.x, self.end.x)
        y = abs(self.end.y - self.start.y)/2 + min(self.start.y, self.end.y)
        z = abs(self.end.z - self.start.z)/2 + min(self.start.z, self.end.z)
        pt = Point(x,y,z)
        return pt
    def getDirectionVector(self):
        unitVec=[0,0,0]
        if(self.start != self.end):
            dx = self.end.x - self.start.x
            dy = self.end.y - self.start.y
            dz = self.end.z - self.start.z
            norm    = math.sqrt(dx**2 + dy**2 + dz**2)
            unitVec = [dx/norm, dy/norm, dz/norm]
        return unitVec
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
    def isEqual(self, line):
        return self.start == line.start and self.end == line.end
    def isParallel(self, line):
        curVector = self.getDirectionVector()
        lineVc    = line.getDirectionVector()
        cross_product = (curVector[1] * lineVc[2] - curVector[2] * lineVc[1], 
                         curVector[2] * lineVc[0] - curVector[0] * lineVc[2], 
                         curVector[0] * lineVc[1] - curVector[1] * lineVc[0])
    
        #  如果叉积为零向量，则向量平行
        return all(component == 0 for component in cross_product)
     
    def isCover(self, line):
        if self.isParallel(line) == False:
            return False
        tmpLine = LineString([(self.start.x, self.start.y, self.start.z), (self.end.x, self.end.y, self.end.z)])
        spLine  = LineString([(line.start.x, line.start.y, line.start.z), (line.end.x, line.end.y, line.end.z)])
        return spLine.intersects(tmpLine) 
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
        self.similarity = 0.0

    def setSimilarity(self, sim):
        self.similarity = sim

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
    def getCenterPoints(self):
        pts=[]
        if len(self.segs) == 1:
            pts.append((self.segs[0].start.x, self.segs[0].start.y, self.segs[0].start.z))
            pts.append((self.segs[0].end.x, self.segs[0].end.y, self.segs[0].end.z))
        elif len(self.segs) >1:
            for i in range(len(self.segs)-1):
                seg1 = self.segs[i]
                seg2 = self.segs[i+1]
                if i == (len(self.segs)-2):
                    pts.append((seg1.start.x, seg1.start.y, seg1.start.z)) 
                    if seg1.end != seg2.start:
                        pts.append((seg1.end.x, seg1.end.y, seg1.end.z))
                    pts.append((seg2.start.x, seg2.start.y, seg2.start.z)) 
                    pts.append((seg2.end.x, seg2.end.y, seg2.end.z))
                else:
                    pts.append((seg1.start.x, seg1.start.y, seg1.start.z))
                    if seg1.end != seg2.start:
                        pts.append((seg1.end.x, seg1.end.y, seg1.end.z))
        return pts

                         
class MikamiTauchi:
    def __init__(self, rtGraph:Grid3D, enMultiLyr:bool = True):
        self.rtGraph:Grid3D    = rtGraph
        self.allPointsDT:dict  = dict()   #store(state[0,0,0]) : x, y, z 0:1 
        self.allLines:dict     = dict()
        self.startSet:LineContainer  = LineContainer()
        self.targetSet:LineContainer = LineContainer()
        self.smrWeight      = 3
        self.cstWeight      = 1
        self.enMultiLyr     = enMultiLyr
        self.allCandidatePaths = []
        self.path:Path      = None
    def _isValid(self, pt:Point):
        return self.rtGraph.isValid((pt.x, pt.y, pt.z))
    def _isOccupied(self, pt:Point):
        return self.rtGraph.is_occupied((pt.x, pt.y, pt.z))
    def __sortLambdaTp(self, line:Line, template:list):
        #find intersects line
        segs = []
        for i in range(len(template)-1):
            s = template[i]
            t = template[i+1]
            tmpLine = Line(Point(s[0], s[1], s[2]), Point(t[0], t[1], t[2]))
            segs.append(tmpLine)
        
        coverLine = line
        for tl in segs:
            if line.isCover(tl):
                coverLine = tl
                break
        centerPoint = coverLine.getCentroid()
        return self.__sortLambda(line, centerPoint)
    def __sortLambda(self, line:Line, pt:Point):
        if(line.axisDirect == AxisEnum.AXIS_X):
            return abs(pt.y - line.start.y)
        if(line.axisDirect == AxisEnum.AXIS_Y):
            return abs(pt.x - line.start.x)
        if(line.axisDirect == AxisEnum.AXIS_Z):
            return abs(pt.x - line.start.x) + abs(pt.y - line.start.y)
    def __getUnitDirectionVector(self, startPt, endPt):
        """
        #get unit vector
        start: (x, y, z)
        end  : (x, y, z)
        """
        norm    = math.sqrt((endPt[0]-startPt[0])**2 + (endPt[1]-startPt[1])**2 + (endPt[2]-startPt[2])**2)
        unitVec = ((endPt[0]-startPt[0])/norm, (endPt[1]-startPt[1])/norm, (endPt[2]-startPt[2])/norm)
        return unitVec
    def __topologyExtract(self, ctl):
        topo = []
        unitVec = []
        for i in range(len(ctl)-1):
            s = ctl[i]
            t = ctl[i+1]
            unitVec.append(self.__getUnitDirectionVector(s, t))
        # print("unitVec:", unitVec)
        for vec in unitVec:
            if vec == (1,0,0):
                topo.append(TopologyEnum.RIGHT)
            elif vec == (-1, 0, 0):
                topo.append(TopologyEnum.LEFT)
            elif vec == (0, 1, 0):
                topo.append(TopologyEnum.UP)
            elif vec == (0, -1, 0):
                topo.append(TopologyEnum.DOWN)
            elif vec == (0, 0, 1):
                topo.append(TopologyEnum.BACK)
            elif vec == (0, 0,-1):
                topo.append(TopologyEnum.FRONT)
            else:
                pass
        return topo 
    def __getMirrorTopo(self, topo:list=[]):
        tp = []
        for t in topo:
            if t.value ==0 or t.value ==1:
                tp.append(TopologyEnum(abs(1-t.value)))
            if t.value ==2 or t.value ==3:
                tp.append(TopologyEnum(abs(5-t.value)))
            if t.value ==4 or t.value ==5:
                tp.append(TopologyEnum(abs(9-t.value)))
        return tp    
    def __calSimilarity(self, path:Path, template:list):
        orgTp = self.__topologyExtract(template)

        centerLine = path.getCenterPoints()
        # print("\ncenterLine:{}".format(centerLine))
        resTp = self.__topologyExtract(centerLine)

        mLen = min(len(orgTp), len(resTp))
        count = 0
        for i in range(mLen):
            if(orgTp[i] == resTp[i]):
                count += 1
            else:
                break
        simValue = 0.0
        if(len(orgTp) > len(resTp)):
            simValue = count/len(orgTp)
        else:
            simValue = count/len(resTp)
        print("Similarity: {}\n".format(simValue))
        return simValue    
    def __beExtendInAxis(self, ePt: Point, dir:  AxisEnum):
        return (ePt in self.allPointsDT and self.allPointsDT[ePt][dir.value] == 1)  
    def __duplicateProcess(self, ln: Line):
        if ln !=None:
            for point in ln.pointSet:
                state = [0,0,0]
                if point not in self.allPointsDT:
                    state[ln.axisDirect.value] =  1
                    self.allPointsDT[point] = state
                else:
                    self.allPointsDT[point][ln.axisDirect.value] = 1
    def __extendNoDuplicate(self, pLine:Line, axisDir:AxisEnum,  tpDir:TopologyEnum=TopologyEnum.ANY):
        lines = []
        for pt in pLine.pointSet:
            if pt == pLine.origin:
                continue
            if  self.__beExtendInAxis(pt, axisDir) == False:
                _,line = self.createLine(pt, axisDir, tpDir)
                if line !=None:
                    line.setParent(pLine)
                    lines.append(line)
                    self.__duplicateProcess(line)  
        return lines
    def __extend(self, pLine:Line, axisDir:AxisEnum,  tpDir:TopologyEnum=TopologyEnum.ANY):
        lines = []
        for pt in pLine.pointSet:
            if pt == pLine.origin:
                continue
            _,line = self.createLine(pt, axisDir, tpDir)
            if line !=None:
                line.setParent(pLine)
                lines.append(line)
                self.__duplicateProcess(line)  
        return lines
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
            if(start == end):
                continue
            ln  = Line(start, end)
            path.addSeg(ln)
        return  path
    def getBestLengthPath(self, lineSet:list, targetContainer:LineContainer):
        cdtPaths = self.calIntersection(lineSet, targetContainer)
        bestPath:Path = None
        bestLen  = math.inf
        for path in cdtPaths:
            wireLen = path.getLength()
            if(wireLen < bestLen):
                bestPath = path
                bestLen = wireLen
        return bestPath
    def getSimilarityPath(self, lineSet:list, template:list, targetContainer:LineContainer):
        cdtPaths = self.calIntersection(lineSet, targetContainer)
        if len(cdtPaths)==0:
            return None
        else:
            self.allCandidatePaths.extend(cdtPaths)
            for path in cdtPaths:
                sim = self.__calSimilarity(path, template)
                path.setSimilarity(sim)
            cdtPaths = sorted(cdtPaths, key= lambda pt: pt.similarity,reverse=True) 
            return cdtPaths[0]
    def calIntersection(self, lineSet:list, targetContainer:LineContainer)->Path:
        cdtPaths=[]
        for line1 in lineSet:
            lines = targetContainer.queryIntersect(line1)
            if(len(lines) == 0):
                continue
            else:
                for ll in lines:
                    path = self.getPath(line1, ll)
                    cdtPaths.append(path)
        return cdtPaths
 

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
        
        path =  self.getBestLengthPath(startLines, self.targetSet)
        while len(startLines) !=0 and len(goalLines)!=0 and path ==None:
            #expand S
            expandList = []
            for subLine in startLines:
                e = self.expandLineWithTp(subLine, self.startSet)
                e = sorted(e, key= lambda line: self.__sortLambda(line, goal))
                expandList.extend(e)
            self.startSet.setLines(expandList)
            visitSource.append(expandList)   
            path  = self.getBestLengthPath(expandList, self.targetSet)
            if (path != None):
                print("Find path success from source !")
                break
            startLines =   expandList

            #expand T
            expandListT = []
            for subLine in goalLines:
                e = self.expandLineWithTp(subLine, self.targetSet)
                e = sorted(e, key= lambda line: self.__sortLambda(line, goal))
                expandListT.extend(e) 
            self.targetSet.setLines(expandListT)
            visitTarget.append(expandListT)
            path  = self.getBestLengthPath(expandListT, self.startSet)
            if (path != None):
                print("Find path success from target !")
                break
            goalLines = expandListT
        
        self.showAlgoResult(visitSource, visitTarget, path)
        plt.show()
    def dfs(self, traversed:list, curLine:Line,  start: Point, goal:Point, sTopoList:list, template:list, level):
        curTopoList = copy.deepcopy(sTopoList)  
        tp = TopologyEnum.ANY
        print("curLevel:{0}, len(topList):{1}".format(level, len(curTopoList)))
        if len(curTopoList) != 0:
            tp = curTopoList.pop(0)    
        print("currentTopo:{}".format(tp))
        if curLine not in traversed:
            if self.path != None:
                return
            #find Path
            path  = self.getSimilarityPath([curLine], template, self.targetSet)
            if path != None and len(curTopoList)==0:
                self.path = path
                print("find Path")
                return 
            
            if  len(curTopoList)==0:
                return
            if level > len(template):
                return
            traversed.append(curLine)

            #expand
            epdLines = self.expandLineWithTp(curLine, self.startSet, tp)
            segs = []
            for i in range(len(template)-1):
                s = template[i]
                t = template[i+1]
                tmpLine = Line(Point(s[0], s[1], s[2]), Point(t[0], t[1], t[2]))
                segs.append(tmpLine)
            refPoint = segs[level+1].getCentroid()
            minDis      = math.inf
            for cdtLine in epdLines:
                for tl in segs:
                    if cdtLine.isCover(tl):          
                        tmpPt  = tl.getCentroid()
                        dis = abs(tmpPt.z - start.z) + abs(tmpPt.y - start.y) + abs(tmpPt.x - start.x)
                        if  dis < minDis:
                            minDis = dis
                            refPoint = tmpPt            
            epdLines  = sorted(epdLines, key= lambda line: self.__sortLambda(line, refPoint))
            level+=1
 
            # traverse
            for nextLine in epdLines:
                self.dfs(traversed, nextLine, start, goal, curTopoList, template, level)

            
    def findPathWithTemplate(self, s, e, template:list=[]):
        #calculate topology
        topo = self.__topologyExtract(template)

        #tmp
        start = Point(s[0], s[1], s[2])
        goal  = Point(e[0], e[1], e[2])

        # Invalid Judgement
        if self.rtGraph.is_occupied((start.x, start.y, start.z)):
            print("Start is occupied!")
            return []
        if self.rtGraph.is_occupied((goal.x, goal.y, goal.z)):
            print("Target is occupied")
            return []
        
        #topo proccess
        mirroTopo = self.__getMirrorTopo(topo)
        print("Topo", topo)
        print("mirroTopo", mirroTopo)

        sTopoList = [TopologyEnum.ANY]
        tTopoList = [TopologyEnum.ANY]
        if(len(topo) ==  1):
            sTopoList = copy.deepcopy(topo)
            tTopoList = copy.deepcopy(mirroTopo)
        else:
            mid = int(len(topo)/2)
            sTopoList = topo[0:mid]
            tTopoList = mirroTopo[mid: len(topo)]
            tTopoList = tTopoList[::-1]
      
        sTmpList  = copy.deepcopy(template)
        sTopoList = copy.deepcopy(topo)
        
  

        #create s/t line
        level = 0
        startLines     =  self.createLineAll(start, sTopoList.pop(0))
        self.startSet.setLines(startLines)
        for l in startLines:
            l.draw('red')
        goalLines   = self.createLineAll(goal, tTopoList.pop(0))
        self.targetSet.setLines(goalLines)
        for l in goalLines:
            l.draw('green') 
        
        #traverse
        traverseList = [ ]
        self.dfs(traverseList, startLines[0], start, goal, sTopoList, sTmpList, level)


        if self.path == None:
            print("Find Path failed with template! start find without template!")
            self.findPath(s,e)
        else:
            self.showAlgoResult([traverseList], [[]], self.path)
            plt.show()
 
    def createLineAll(self, coord: Point, tpDir:TopologyEnum=TopologyEnum.ANY):
        lines = []
        _,l1 = self.createLine(coord, AxisEnum.AXIS_X, tpDir)
        _,l2 = self.createLine(coord, AxisEnum.AXIS_Y, tpDir)
        
        if l1 !=None:
            lines.append(l1)
        if l2 !=None:
            lines.append(l2)

        if self.enMultiLyr ==True:
            _,l3 = self.createLine(coord, AxisEnum.AXIS_Z, tpDir)
            if l3 !=None:
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
            if (tpDir == TopologyEnum.ANY or tpDir == TopologyEnum.RIGHT):
                for x in range(coord.x, self.rtGraph.w):
                    tmpPoint = Point(x, coord.y, coord.z)
                    if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                        break
                    points.add(tmpPoint)
            if (tpDir == TopologyEnum.ANY or tpDir == TopologyEnum.LEFT):
                for x in range(coord.x, -1, -1):
                    tmpPoint = Point(x, coord.y, coord.z)
                    if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                        break
                    points.add(tmpPoint)
            if len(points) != 0:
                points_list = list(points)
                points_list.sort(key=lambda point: point.x)
                bigger =  max([pt.x for pt in points_list])
                small  =  min([pt.x for pt in points_list])
                line.setStart(Point(small, coord.y, coord.z))
                line.setEnd(Point(bigger, coord.y, coord.z))
                for pt in points_list:
                    line.addPoint(pt)
            else:
                line = None 
        elif AxisEnum.AXIS_Y == axis:
            points = set()
            if (tpDir == TopologyEnum.ANY or tpDir == TopologyEnum.UP):
                for y in range(coord.y, self.rtGraph.h):
                    tmpPoint = Point(coord.x, y, coord.z)
                    if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                        break
                    points.add(tmpPoint)
            if (tpDir == TopologyEnum.ANY or tpDir == TopologyEnum.DOWN):
                for y in range(coord.y, -1, -1):
                    tmpPoint = Point(coord.x, y, coord.z)
                    if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                        break
                    points.add(tmpPoint)
            if len(points) != 0:
                points_list = list(points)
                points_list.sort(key=lambda point: point.y)
                bigger =  max([pt.y for pt in points_list])
                small =  min([pt.y for pt in points_list])
                line.setStart(Point(coord.x, small, coord.z))
                line.setEnd(Point(coord.x, bigger, coord.z))
                for pt in points_list:
                    line.addPoint(pt)
            else:
                line = None 
        elif AxisEnum.AXIS_Z == axis and self.enMultiLyr == True:
            points = set()
            if (tpDir == TopologyEnum.ANY or tpDir == TopologyEnum.BACK):
                for z in range(coord.z, self.rtGraph.l):
                    tmpPoint = Point(coord.x, coord.y, z)
                    if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                        break
                    points.add(tmpPoint)
            if (tpDir == TopologyEnum.ANY or tpDir == TopologyEnum.FRONT):
                for z in range(coord.z, -1, -1):
                    tmpPoint = Point(coord.x, coord.y, z)
                    if (self._isValid(tmpPoint) == False or self._isOccupied(tmpPoint)== True):
                        break
                    points.add(tmpPoint)
            if len(points) != 0:
                points_list = list(points)
                points_list.sort(key=lambda point: point.z)
                bigger =  max([pt.z for pt in points_list])
                small  =  min([pt.z for pt in points_list])
                line.setStart(Point(coord.x, coord.y, small))
                line.setEnd(Point(coord.x, coord.y, bigger))
                for pt in points_list:
                    line.addPoint(pt)
            else:
                line = None 
        else:
            line = None 

        if( line != None):
            id = len(self.allLines)
            line.setId(id)
            self.allLines[id] = line
            return (True, line)
        else:
            return (False, None)
    

    def expandLineWithTp(self, line:Line, lct:LineContainer, tpDir:TopologyEnum=TopologyEnum.ANY):
        #record expanded points, if expand ignore
        lines  = []
        if line.axisDirect == AxisEnum.AXIS_X:
            l1 = self.__extend(line, AxisEnum.AXIS_Y, tpDir) 
            lines.extend(l1)
            if self.enMultiLyr == True:
                l2 = self.__extend(line, AxisEnum.AXIS_Z, tpDir)
                lines.extend(l2)
                             
        elif line.axisDirect == AxisEnum.AXIS_Y:
            l1 = self.__extend(line, AxisEnum.AXIS_X, tpDir) 
            lines.extend(l1)
            if self.enMultiLyr == True:
                l2 = self.__extend(line, AxisEnum.AXIS_Z, tpDir)
                lines.extend(l2)
          
          
        elif (line.axisDirect == AxisEnum.AXIS_Z and self.enMultiLyr == True):
            l1 = self.__extend(line, AxisEnum.AXIS_X, tpDir) 
            l2 = self.__extend(line, AxisEnum.AXIS_Y, tpDir)
            lines.extend(l1)
            lines.extend(l2)
        return lines
                
    def expandLine(self, line:Line, lct:LineContainer, tpDir:TopologyEnum=TopologyEnum.ANY):
        #record expanded points, if expand ignore
        lines  = []
        if line.axisDirect == AxisEnum.AXIS_X:
            l1 = self.__extendNoDuplicate(line, AxisEnum.AXIS_Y, tpDir) 
            lines.extend(l1)
            if self.enMultiLyr == True:
                l2 = self.__extendNoDuplicate(line, AxisEnum.AXIS_Z, tpDir)
                lines.extend(l2)
                             
        elif line.axisDirect == AxisEnum.AXIS_Y:
            l1 = self.__extendNoDuplicate(line, AxisEnum.AXIS_X, tpDir) 
            lines.extend(l1)
            if self.enMultiLyr == True:
                l2 = self.__extendNoDuplicate(line, AxisEnum.AXIS_Z, tpDir)
                lines.extend(l2)
          
          
        elif (line.axisDirect == AxisEnum.AXIS_Z and self.enMultiLyr == True):
            l1 = self.__extendNoDuplicate(line, AxisEnum.AXIS_X, tpDir) 
            l2 = self.__extendNoDuplicate(line, AxisEnum.AXIS_Y, tpDir)

            lines.extend(l1)
            lines.extend(l2)
          
    
        return lines
    
    def showAlgoResult(self, sourceList:list, targetList:list, path):
        for lst in sourceList:
            for l in lst:
                l.draw('gray')
        for lst in targetList:
            for l in lst:
                l.draw('gray')
        
        if path != None:
            print("Path similarity: ", path.similarity)
            path.draw()
        