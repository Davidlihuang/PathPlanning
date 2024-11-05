
# coding=utf-8

from __future__ import division
from __future__ import absolute_import
import os
import sys
import math
import copy
import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

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
    
class Point(object):
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

class Line(object): 
    def __init__(self, st, ed):
        self.id      = -1
        self.origin  = st               #current line origin point
        self.start   = st           #start point
        self.end     = ed           #end   point
        self.parent   = None         #{Line:Line}
        self.pointSet      = []
        self.axisDirect    = AxisEnum.AXIS_OTHER
        self.lineDirect    = TopologyEnum.ANY
    
    def printLines(self):
        print "origin:(", self.origin.x , self.origin.y, self.origin.z , ")"
        print "start:(", self.start.x , self.start.y, self.start.z , ")"
        print "end:(", self.end.x , self.end.y, self.end.z , ")\n"
     
    def setId(self, id):
        self.id = id
    def setOrigin(self, origin):
        self.origin   = origin  
    def setStart(self, pt):
        self.start = pt
    def setEnd(self, pt):
        self.end = pt    
    def setParent(self, line):
        self.parent  = line
    def setAxisType(self, axis):
        self.axisDirect = axis
    def setLineDirect(self, topo):
        self.lineDirect = topo
    def setAxisPoints(self, points, axis):
        for p in points:
            self.addPoint(p, axis)
    def addPoint(self, pt):     
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
            if abs(norm) < 1e-5:
                return None
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
    def isSelfEqual(self):
        if (self.start.x == self.end.x and 
            self.start.y == self.end.y and
            self.start.z == self.end.z):
            return True
        else:
            return False
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
    
    # ---------------------------------------------------------------------------------------- #
    #  判断两条线段是否共面  （任取两条线段的三个端点，计算这三个端点中随意构成的两个向量的叉积N【叉积垂直于三点构成的平面】；用叉积N与未曾参与计算的那个端点所在的向量做点积 若点积为0则两条线段共面）
    # ---------------------------------------------------------------------------------------- #
    # V1[[x1,y1, z1], [x2, y2, z2]]
    # 计算两点表示的两个三维向量的叉积  N = [NX2-NX1, NY2-NY1, NZ2-NZ1]
    def CrossProduct3D(self, V1, V2):
        '''
        计算端点表示的三维向量的叉积
        :param V1: 两端点表示的向量  例如 V1=[[x1,y1, z1], [x2, y2, z2]]
        :param V2: 两端点表示的向量  例如 V2=[[x1,y1, z1], [x2, y2, z2]]
        :return: 两向量的叉积, 向量坐标表示的向量 例如 N = [Xn1, Yn1, Zn1]  (其中Xn1 = x2-x1 若N=[[x1,y1, z1], [x2, y2, z2]]）
        '''
        N = [(V1[1][1]-V1[0][1])*(V2[1][2]-V2[0][2]) - (V1[1][2]-V1[0][2])*(V2[1][1]-V2[0][1]),
            -((V1[1][2]-V1[0][2])*(V2[1][0]-V2[0][0]) - (V1[1][0]-V1[0][0])*(V2[1][2]-V2[0][2])),
            (V1[1][0]-V1[0][0])*(V2[1][1]-V2[0][1]) - (V1[1][1]-V1[0][1])*(V2[1][0]-V2[0][0])]
        return N

    # V1[V1X, V1Y, V1Z]
    # 计算坐标表示的两个三维向量的点积
    def DotProduct3D(self, V1, V2):
        '''
        计算向量坐标表示的三维向量的点积
        :param V1: 向量坐标表示的向量 例如 V1 = [Xv1, Yv1, Zv1]  (其中Xv1 = x2-x1 若V1=[[x1,y1, z1], [x2, y2, z2]]）
        :param V2: 向量坐标表示的向量 例如 V2 = [Xv2, Yv2, Zv2]
        :return: 两向量的点积，数值
        '''
        Dot = V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]
        return Dot

    # V1[[x1,y1, z1], [x2, y2, z2]]
    # V2[[x1,y1, z1], [x2, y2, z2]]
    # 判断两条线段是否共面
    def twoLineSegmentCoplanar3D(self, V1, V2):
        '''
        判断三维空间中的两条线段是否共面
        :param V1: 两端点表示的向量  例如 V1=[[x1,y1, z1], [x2, y2, z2]]
        :param V2: 两端点表示的向量  例如 V2=[[x1,y1, z1], [x2, y2, z2]]
        :return: 若共面则True 否则False
        '''
        tempV1 = V1
        tempV2 = [V1[0], V2[1]]
        N = self.CrossProduct3D(tempV1, tempV2)
        V2p = [V2[1][0]-V2[0][0], V2[1][1]-V2[0][1], V2[1][2]-V2[0][2]]
        if abs(self.DotProduct3D(N, V2p)) <0.0001:
            return True
        else:
            return False

 
    # 三维空间中快速排斥,快速排斥法  排除一部分不满足条件的线段
    # Vo[[x1,y1, z1], [x2, y2, z2]]
    def RapidRepel(self, Vo, Vp):
        '''
        快速排斥法，排除三维空间中不可能相交的线段
        :param Vo: 两端点表示的向量  例如 Vo=[[x1,y1, z1], [x2, y2, z2]]
        :param Vp: 两端点表示的向量  例如 Vp=[[x1,y1, z1], [x2, y2, z2]]
        :return: 若通过快速排斥 则表示两线段可能相交 返回True; 否则 False
        '''
        if (        max(Vo[0][0], Vo[1][0]) >= min(Vp[0][0], Vp[1][0])
                and min(Vo[0][0], Vo[1][0]) <= max(Vp[0][0], Vp[1][0])
                and max(Vo[0][1], Vo[1][1]) >= min(Vp[0][1], Vp[1][1])
                and min(Vo[0][1], Vo[1][1]) <= max(Vp[0][1], Vp[1][1])
                and max(Vo[0][2], Vo[1][2]) >= min(Vp[0][2], Vp[1][2])
                and min(Vo[0][2], Vo[1][2]) <= max(Vp[0][2], Vp[1][2])
        ):
            return True
        else:
            return False


    # ---------------------------------------------------------------------------------------- #
    #  跨立实验  若满足前面的两个条件  且通过跨立实验   则认为这两条线段是相交的  (这里先计算以一条线段的一个端点为起点与另一条线段的一个端点组成的向量tempV1，与该点为起点线段的向量的叉积N1；
    #  再计算该点为起点线段的向量 与 该点为起点与另一条线段的另一个端点组成的向量TempV2 的叉积N2 计算N1、N2的点积 再以另一条线段为中间向量再进行计算 若两次计算的结果均大于0 则认为两条线段相交)
    # ---------------------------------------------------------------------------------------- #
    def StraddleExperiment(self, V1, V2):
        '''
        跨立实验，排除三维空间中不可能相交的线段
        :param V1: 两端点表示的向量  例如 V1=[[x1,y1, z1], [x2, y2, z2]]
        :param V2: 两端点表示的向量  例如 V2=[[x1,y1, z1], [x2, y2, z2]]
        :return: 通过跨立实验 返回True，否则返回False
        '''
        tempV1 = [V1[0], V2[1]]
        tempV2 = [V1[0], V2[0]]
        N1 = self.CrossProduct3D(tempV1, V1)
        N2 = self.CrossProduct3D(V1, tempV2)
        res1 = self.DotProduct3D(N1, N2)

        tempV3 = [V2[0], V1[1]]
        tempV4 = [V2[0], V1[0]]
        N3 = self.CrossProduct3D(tempV3, V2)
        N4 = self.CrossProduct3D(V2, tempV4)
        res2 = self.DotProduct3D(N3, N4)

        if res1 > 0 and res2>0:
            return True
        else:
            return False
    
    def isIntersect(self, line2):
        pt = None
        V1 = [[self.start.x, self.start.y, self.start.z], [self.end.x, self.end.y, self.end.z]]
        V2 = [[line2.start.x, line2.start.y, line2.start.z], [line2.end.x, line2.end.y, line2.end.z]]
        if self.twoLineSegmentCoplanar3D(V1, V2) and self.RapidRepel(V1, V2):
            if self.StraddleExperiment(V1, V2):
                # 线段的方向向量
                p1 = [ self.start.x,  self.start.y,  self.start.z   ]
                p2 = [ self.end.x,    self.end.y,    self.end.z     ]
                p3 = [ line2.start.x, line2.start.y, line2.start.z]
                p4 = [ line2.end.x,   line2.end.y,   line2.end.z  ]
                d1 = [[0,0,0], [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]]
                d2 = [[0,0,0], [p4[0] - p3[0], p4[1] - p3[1], p4[2] - p3[2]]]
                # 从p1到p3的向量
                p13 = [[0,0,0],[p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]]
                # 计算叉乘
                n = self.CrossProduct3D(d1, d2)
                # 计算点到线段的参数
                t = self.DotProduct3D(self.CrossProduct3D(p13, d2), n) / self.DotProduct3D(self.CrossProduct3D(d1, d2), n)
                s = self.DotProduct3D(self.CrossProduct3D(p13, d1), n) / self.DotProduct3D(self.CrossProduct3D(d1, d2), n)

                # 检查参数是否在0和1之间
                if 0 <= t <= 1 and 0 <= s <= 1:
                    intersection = (p1[0] + t * d1[1][0], p1[1] + t * d1[1][1], p1[2] + t * d1[1][2])
                    pt = Point(intersection[0], intersection[1], intersection[2])
                    return pt
        return pt

    
    def isCover(self, line):        
        if self.isParallel(line) == False:
            return False
        return self.isIntersect(line) != None

    
    def getIntersectionPoint(self, line):
        resPt = self.isIntersect(line)
        if resPt != None:
            return resPt
        else:
            return None
        
    def draw(self,  cr=u'', lw=1, lsty=u'-', ):
        crList = [u'red', u'blue', u'green', u'yellow',u'gold']
        if cr ==u'':
            cr = crList[int(self.start.z)]
        plt.plot([self.start.x, self.end.x],[self.start.y, self.end.y],  linestyle=lsty, linewidth=lw, color=cr)
        for pt in self.pointSet:
            plt.plot(pt.x, pt.y, u'o', markersize=lw+0.5, color=u'Gray')
class LineContainer(object):
    def __init__(self):
        self.lines                = [[],[],[]]     #[[x:Line], [y:Line], [z:Line]] 
        self.container            = [rtree.index.Index(), rtree.index.Index(),rtree.index.Index()]
        self.lineDict             = dict()
    def __calSize(self):
        len = 0
        for lineList in self.lines:
            len += len(lineList)
        return len
    def queryIntersect(self, line):
        itsList = []
        minPoint = Point(min(line.start.x, line.end.x), min(line.start.y, line.end.y), min(line.start.z, line.end.z))
        maxPoint = Point(max(line.start.x, line.end.x), max(line.start.y, line.end.y), max(line.start.z, line.end.z))

        for item in self.container:
            intersecting_segments = item.intersection((minPoint.x, minPoint.y, maxPoint.x, maxPoint.y))
            for sid in intersecting_segments:
                curLine = self.lineDict[sid]
                 
                if curLine.isIntersect(line) != None:
                    itsList.append(curLine)
        return itsList
    
  
    def getLines(self, axis):
        if (axis == AxisEnum.AXIS_X):
            return self.lines[0]
        elif (axis == AxisEnum.AXIS_Y):
            return self.lines[1]
        elif (axis == AxisEnum.AXIS_Z):
            return self.lines[2]
        else:
            return []
        
    def setLines(self, lines):
        for line in lines:
            self.addLine(line, line.axisDirect)

    def addLine(self, line, axis):
        if(axis < AxisEnum.AXIS_X or axis > AxisEnum.AXIS_Z):
            print u"Error: invalid index:{}".format(axis)
            return

        self.lines[axis].append(line)
        self.lineDict[line.id] = line
        
        #add to RTree
        minPoint = Point(min(line.start.x, line.end.x), min(line.start.y, line.end.y), min(line.start.z, line.end.z))
        maxPoint = Point(max(line.start.x, line.end.x), max(line.start.y, line.end.y), max(line.start.z, line.end.z))
        self.container[axis].insert(int(line.id), (minPoint.x, minPoint.y, maxPoint.x, maxPoint.y), obj=line)

class Path(object):
    def __init__(self):
        self.segs = []
        self.vias = []
        self.shapes = []
        self.similarity = 0.0

    def setSimilarity(self, sim):
        self.similarity = sim

    def addSeg(self, seg):
        self.segs.append(seg)
    def getLength(self):
        tLen = 0.0
        for seg in self.segs:
            tLen += seg.getLength()
        return tLen
    def draw(self, cr=u'', width = 6):
        for seg in self.segs:
            seg.draw(cr, width) 
    def printSegs(self):
        for seg in self.segs:
            print seg.printLines()
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

                         
class MikamiTauchi(object):
    def __init__(self, rtGraph, enMultiLyr = True):
        self.rtGraph    = rtGraph
        self.allPointsDT  = dict()   #store(state[0,0,0]) : x, y, z 0:1 
        self.allLines     = dict()
        self.startSet     = LineContainer()
        self.targetSet    = LineContainer()
        self.smrWeight      = 3
        self.cstWeight      = 1
        self.enMultiLyr     = enMultiLyr
        self.allCandidatePaths = []
        self.path      = None
    def _isValid(self, pt):
        return self.rtGraph.isValid((pt.x, pt.y, pt.z))
    def _isOccupied(self, pt):
        return self.rtGraph.is_occupied((pt.x, pt.y, pt.z))
    def __sortLambdaTp(self, line, template):
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
    def __sortLambda(self, line, pt):
        if(line.axisDirect == AxisEnum.AXIS_X):
            return abs(pt.y - line.start.y)
        if(line.axisDirect == AxisEnum.AXIS_Y):
            return abs(pt.x - line.start.x)
        if(line.axisDirect == AxisEnum.AXIS_Z):
            return abs(pt.x - line.start.x) + abs(pt.y - line.start.y)
    def __getUnitDirectionVector(self, startPt, endPt):
        u"""
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
    def __getMirrorTopo(self, topo=[]):
        tp = []
        for t in topo:
            if t == TopologyEnum.LEFT:
                tp.append(TopologyEnum.RIGHT)
            elif t == TopologyEnum.RIGHT:
                tp.append(TopologyEnum.LEFT)

            if t == TopologyEnum.UP:
                tp.append(TopologyEnum.DOWN)
            elif t == TopologyEnum.DOWN:
                tp.append(TopologyEnum.UP)


            if t == TopologyEnum.FRONT:
                tp.append(TopologyEnum.BACK)
            elif t == TopologyEnum.BACK:
                tp.append(TopologyEnum.FRONT)
        print "tp:", tp
        return tp    
    def __calSimilarity(self, path, template):
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
        print u"Similarity: {}\n".format(simValue)
        return simValue    
    def __beExtendInAxis(self, ePt, dir):
        return (ePt in self.allPointsDT and self.allPointsDT[ePt][dir] == 1)  
    def __duplicateProcess(self, ln):
        if ln !=None:
            for point in ln.pointSet:
                state = [0,0,0]
                if point not in self.allPointsDT:
                    state[ln.axisDirect] =  1
                    self.allPointsDT[point] = state
                else:
                    self.allPointsDT[point][ln.axisDirect] = 1
    def __extendNoDuplicate(self, pLine, axisDir,  tpDir=TopologyEnum.ANY):
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
    def __extend(self, pLine, axisDir,  tpDir=TopologyEnum.ANY):
        lines = []
        for pt in pLine.pointSet:
            if pt == pLine.origin:
                continue
            _,line = self.createLine(pt, axisDir, tpDir)
            if line !=None:
                line.setParent(pLine)
                line.setOrigin(pt)
                lines.append(line)
                self.__duplicateProcess(line)  
        return lines
    def getPath(self, line1, line2):
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
    def getBestLengthPath(self, lineSet, targetContainer):
        cdtPaths = self.calIntersection(lineSet, targetContainer)
        bestPath = None
        bestLen  =  float('inf')
        for path in cdtPaths:
            wireLen = path.getLength()
            if(wireLen < bestLen):
                bestPath = path
                bestLen = wireLen
      
        return bestPath
    def getSimilarityPath(self, lineSet, template, targetContainer):
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
    def calIntersection(self, lineSet, targetContainer):
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
 
    
    def findPath(self, s, e, tpS = TopologyEnum.ANY, tpT = TopologyEnum.ANY, ):
        #tmp
        visitSource = []
        visitTarget = []

        start = Point(s[0], s[1], s[2])
        goal  = Point(e[0], e[1], e[2])
        # invalid
        if self.rtGraph.is_occupied((start.x, start.y, start.z)):
            print u"Start is occupied!"
            return []
        if self.rtGraph.is_occupied((goal.x, goal.y, goal.z)):
            print u"Target is occupied"
            return []
        
        #create s/t line
        startLines     =  self.createLineAll(start, tpS)
        self.startSet.setLines(startLines)
        for l in startLines:
            l.draw(u'red')
        goalLines   = self.createLineAll(goal, tpT)
        self.targetSet.setLines(goalLines)
        for l in goalLines:
            l.draw(u'green') 
        
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
                self.path = path
                print u"Find path success from source !"
                break
            startLines =   expandList

            #expand T
            expandListT = []
            for subLine in goalLines:
                e = self.expandLineWithTp(subLine, self.targetSet)
                e = sorted(e, key= lambda line: self.__sortLambda(line, start))
                expandListT.extend(e) 
            self.targetSet.setLines(expandListT)
            visitTarget.append(expandListT)
            path  = self.getBestLengthPath(expandListT, self.startSet)
            if (path != None):
                self.path = path
                print u"Find path success from target !"
                break
            goalLines = expandListT
        
        # self.showAlgoResult(visitSource, visitTarget, path)
        
    def dfs(self, traversed, curLine,  start, goal, sTopoList, template, level):
        curTopoList = copy.deepcopy(sTopoList)  
        tp = TopologyEnum.ANY
        print u"curLevel:{0}, len(topList):{1}".format(level, len(curTopoList))
        if len(curTopoList) != 0:
            tp = curTopoList.pop(0)    
        print u"currentTopo:{}".format(tp)
        if curLine not in traversed:
            if self.path != None:
                return
            #find Path
            path  = self.getSimilarityPath([curLine], template, self.targetSet)
            if path != None and len(curTopoList)==0:
                self.path = path
                print u"find Path"
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
            minDis      =  float('inf')
            for cdtLine in epdLines:
                # cdtLine.printLine() 
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

            
    def findPathWithTemplate(self, s, e, template=[]):
        #calculate topology
        topo = self.__topologyExtract(template)

        #tmp
        start = Point(s[0], s[1], s[2])
        goal  = Point(e[0], e[1], e[2])

        # Invalid Judgement
        if self.rtGraph.is_occupied((start.x, start.y, start.z)):
            print u"Start is occupied!"
            return []
        if self.rtGraph.is_occupied((goal.x, goal.y, goal.z)):
            print u"Target is occupied"
            return []
        
        #topo proccess
        mirroTopo = self.__getMirrorTopo(topo)
        print u"Topo", topo
        print u"mirroTopo", mirroTopo
        

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
            l.draw(u'red')
        goalLines   = self.createLineAll(goal, tTopoList[0])
        self.targetSet.setLines(goalLines)
        for l in goalLines:
            l.draw(u'green') 
        
        #traverse
        traverseList = [ ]
        self.dfs(traverseList, startLines[0], start, goal, sTopoList, sTmpList, level)


        if self.path == None:
            print u"Find Path failed with template! start find without template!"
            self.findPath(s,e, topo[0], tTopoList[0])
            self.path.similarity = self.__calSimilarity(self.path, sTmpList)
            print "Path: ", self.path.printSegs()
            self.showAlgoResult([traverseList], [[]], self.path)
            plt.show()
        else:
            self.showAlgoResult([traverseList], [[]], self.path)
            plt.show()
 
    def createLineAll(self, coord, tpDir=TopologyEnum.ANY):
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

    def createLine(self, coord, axis, tpDir=TopologyEnum.ANY):
        line = Line(coord, coord)
        if( self._isValid(coord) == False):
            return (False, None)
        
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
        
        
        if( line != None and line.isSelfEqual() == False ):
            if line.start != line.end:
                id = len(self.allLines)
                line.setId(id)
                self.allLines[id] = line
                return (True, line)
      
        return (False, None)       
    

    def expandLineWithTp(self, line, lct, tpDir=TopologyEnum.ANY):
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
                
    def expandLine(self, line, lct, tpDir=TopologyEnum.ANY):
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
    
    def showAlgoResult(self, sourceList, targetList, path):
        for lst in sourceList:
            for l in lst:
                l.draw(u'gray')
        for lst in targetList:
            for l in lst:
                l.draw(u'gray')
        
        if path != None:
            print u"Path similarity: ", path.similarity
            path.draw()
        