# coding=utf-8
from __future__ import division
from __future__ import absolute_import

import math
from enum import Enum
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MatplotlibPolygon
import numpy as np

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
        print("start:({0},{1},{2})".format(self.start.x , self.start.y, self.start.z))
        print("end:({0},{1},{2})".format(self.end.x , self.end.y, self.end.z))
        pass
     
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
    
        #  脠莽鹿没虏忙禄媒脦陋脕茫脧貌脕驴拢卢脭貌脧貌脕驴脝陆脨脨
        return all(component == 0 for component in cross_product)
    
    # ---------------------------------------------------------------------------------------- #
    #  脜脨露脧脕陆脤玫脧脽露脦脢脟路帽鹿虏脙忙  拢篓脠脦脠隆脕陆脤玫脧脽露脦碌脛脠媒赂枚露脣碌茫拢卢录脝脣茫脮芒脠媒赂枚露脣碌茫脰脨脣忙脪芒鹿鹿鲁脡碌脛脕陆赂枚脧貌脕驴碌脛虏忙禄媒N隆戮虏忙禄媒麓鹿脰卤脫脷脠媒碌茫鹿鹿鲁脡碌脛脝陆脙忙隆驴拢禄脫脙虏忙禄媒N脫毛脦麓脭酶虏脦脫毛录脝脣茫碌脛脛脟赂枚露脣碌茫脣霉脭脷碌脛脧貌脕驴脳枚碌茫禄媒 脠么碌茫禄媒脦陋0脭貌脕陆脤玫脧脽露脦鹿虏脙忙拢漏
    # ---------------------------------------------------------------------------------------- #
    # V1[[x1,y1, z1], [x2, y2, z2]]
    # 录脝脣茫脕陆碌茫卤铆脢戮碌脛脕陆赂枚脠媒脦卢脧貌脕驴碌脛虏忙禄媒  N = [NX2-NX1, NY2-NY1, NZ2-NZ1]
    def CrossProduct3D(self, V1, V2):
        '''
        录脝脣茫露脣碌茫卤铆脢戮碌脛脠媒脦卢脧貌脕驴碌脛虏忙禄媒
        :param V1: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 V1=[[x1,y1, z1], [x2, y2, z2]]
        :param V2: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 V2=[[x1,y1, z1], [x2, y2, z2]]
        :return: 脕陆脧貌脕驴碌脛虏忙禄媒, 脧貌脕驴脳酶卤锚卤铆脢戮碌脛脧貌脕驴 脌媒脠莽 N = [Xn1, Yn1, Zn1]  (脝盲脰脨Xn1 = x2-x1 脠么N=[[x1,y1, z1], [x2, y2, z2]]拢漏
        '''
        N = [(V1[1][1]-V1[0][1])*(V2[1][2]-V2[0][2]) - (V1[1][2]-V1[0][2])*(V2[1][1]-V2[0][1]),
            -((V1[1][2]-V1[0][2])*(V2[1][0]-V2[0][0]) - (V1[1][0]-V1[0][0])*(V2[1][2]-V2[0][2])),
            (V1[1][0]-V1[0][0])*(V2[1][1]-V2[0][1]) - (V1[1][1]-V1[0][1])*(V2[1][0]-V2[0][0])]
        return N

    # V1[V1X, V1Y, V1Z]
    # 录脝脣茫脳酶卤锚卤铆脢戮碌脛脕陆赂枚脠媒脦卢脧貌脕驴碌脛碌茫禄媒
    def DotProduct3D(self, V1, V2):
        '''
        录脝脣茫脧貌脕驴脳酶卤锚卤铆脢戮碌脛脠媒脦卢脧貌脕驴碌脛碌茫禄媒
        :param V1: 脧貌脕驴脳酶卤锚卤铆脢戮碌脛脧貌脕驴 脌媒脠莽 V1 = [Xv1, Yv1, Zv1]  (脝盲脰脨Xv1 = x2-x1 脠么V1=[[x1,y1, z1], [x2, y2, z2]]拢漏
        :param V2: 脧貌脕驴脳酶卤锚卤铆脢戮碌脛脧貌脕驴 脌媒脠莽 V2 = [Xv2, Yv2, Zv2]
        :return: 脕陆脧貌脕驴碌脛碌茫禄媒拢卢脢媒脰碌
        '''
        Dot = V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]
        return Dot

    # V1[[x1,y1, z1], [x2, y2, z2]]
    # V2[[x1,y1, z1], [x2, y2, z2]]
    # 脜脨露脧脕陆脤玫脧脽露脦脢脟路帽鹿虏脙忙
    def twoLineSegmentCoplanar3D(self, V1, V2):
        '''
        脜脨露脧脠媒脦卢驴脮录盲脰脨碌脛脕陆脤玫脧脽露脦脢脟路帽鹿虏脙忙
        :param V1: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 V1=[[x1,y1, z1], [x2, y2, z2]]
        :param V2: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 V2=[[x1,y1, z1], [x2, y2, z2]]
        :return: 脠么鹿虏脙忙脭貌True 路帽脭貌False
        '''
        tempV1 = V1
        tempV2 = [V1[0], V2[1]]
        N = self.CrossProduct3D(tempV1, tempV2)
        V2p = [V2[1][0]-V2[0][0], V2[1][1]-V2[0][1], V2[1][2]-V2[0][2]]
        if abs(self.DotProduct3D(N, V2p)) <0.0001:
            return True
        else:
            return False

 
    # 脠媒脦卢驴脮录盲脰脨驴矛脣脵脜脜鲁芒,驴矛脣脵脜脜鲁芒路篓  脜脜鲁媒脪禄虏驴路脰虏禄脗煤脳茫脤玫录镁碌脛脧脽露脦
    # Vo[[x1,y1, z1], [x2, y2, z2]]
    def RapidRepel(self, Vo, Vp):
        '''
        驴矛脣脵脜脜鲁芒路篓拢卢脜脜鲁媒脠媒脦卢驴脮录盲脰脨虏禄驴脡脛脺脧脿陆禄碌脛脧脽露脦
        :param Vo: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 Vo=[[x1,y1, z1], [x2, y2, z2]]
        :param Vp: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 Vp=[[x1,y1, z1], [x2, y2, z2]]
        :return: 脠么脥篓鹿媒驴矛脣脵脜脜鲁芒 脭貌卤铆脢戮脕陆脧脽露脦驴脡脛脺脧脿陆禄 路碌禄脴True; 路帽脭貌 False
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
    #  驴莽脕垄脢碌脩茅  脠么脗煤脳茫脟掳脙忙碌脛脕陆赂枚脤玫录镁  脟脪脥篓鹿媒驴莽脕垄脢碌脩茅   脭貌脠脧脦陋脮芒脕陆脤玫脧脽露脦脢脟脧脿陆禄碌脛  (脮芒脌茂脧脠录脝脣茫脪脭脪禄脤玫脧脽露脦碌脛脪禄赂枚露脣碌茫脦陋脝冒碌茫脫毛脕铆脪禄脤玫脧脽露脦碌脛脪禄赂枚露脣碌茫脳茅鲁脡碌脛脧貌脕驴tempV1拢卢脫毛赂脙碌茫脦陋脝冒碌茫脧脽露脦碌脛脧貌脕驴碌脛虏忙禄媒N1拢禄
    #  脭脵录脝脣茫赂脙碌茫脦陋脝冒碌茫脧脽露脦碌脛脧貌脕驴 脫毛 赂脙碌茫脦陋脝冒碌茫脫毛脕铆脪禄脤玫脧脽露脦碌脛脕铆脪禄赂枚露脣碌茫脳茅鲁脡碌脛脧貌脕驴TempV2 碌脛虏忙禄媒N2 录脝脣茫N1隆垄N2碌脛碌茫禄媒 脭脵脪脭脕铆脪禄脤玫脧脽露脦脦陋脰脨录盲脧貌脕驴脭脵陆酶脨脨录脝脣茫 脠么脕陆麓脦录脝脣茫碌脛陆谩鹿没戮霉麓贸脫脷0 脭貌脠脧脦陋脕陆脤玫脧脽露脦脧脿陆禄)
    # ---------------------------------------------------------------------------------------- #
    def StraddleExperiment(self, V1, V2):
        '''
        驴莽脕垄脢碌脩茅拢卢脜脜鲁媒脠媒脦卢驴脮录盲脰脨虏禄驴脡脛脺脧脿陆禄碌脛脧脽露脦
        :param V1: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 V1=[[x1,y1, z1], [x2, y2, z2]]
        :param V2: 脕陆露脣碌茫卤铆脢戮碌脛脧貌脕驴  脌媒脠莽 V2=[[x1,y1, z1], [x2, y2, z2]]
        :return: 脥篓鹿媒驴莽脕垄脢碌脩茅 路碌禄脴True拢卢路帽脭貌路碌禄脴False
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
        print("self:", self)
        V1 = [[self.start.x, self.start.y, self.start.z], [self.end.x, self.end.y, self.end.z]]
        V2 = [[line2.start.x, line2.start.y, line2.start.z], [line2.end.x, line2.end.y, line2.end.z]]
        if self.twoLineSegmentCoplanar3D(V1, V2) and self.RapidRepel(V1, V2):
            if self.StraddleExperiment(V1, V2):
                # 脧脽露脦碌脛路陆脧貌脧貌脕驴
                p1 = [ self.start.x,  self.start.y,  self.start.z   ]
                p2 = [ self.end.x,    self.end.y,    self.end.z     ]
                p3 = [ line2.start.x, line2.start.y, line2.start.z]
                p4 = [ line2.end.x,   line2.end.y,   line2.end.z  ]
                d1 = [[0,0,0], [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]]
                d2 = [[0,0,0], [p4[0] - p3[0], p4[1] - p3[1], p4[2] - p3[2]]]
                # 麓脫p1碌陆p3碌脛脧貌脕驴
                p13 = [[0,0,0],[p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]]
                # 录脝脣茫虏忙鲁脣
                n = self.CrossProduct3D(d1, d2)
                # 录脝脣茫碌茫碌陆脧脽露脦碌脛虏脦脢媒
                t = self.DotProduct3D(self.CrossProduct3D(p13, d2), n) / self.DotProduct3D(self.CrossProduct3D(d1, d2), n)
                s = self.DotProduct3D(self.CrossProduct3D(p13, d1), n) / self.DotProduct3D(self.CrossProduct3D(d1, d2), n)

                # 录矛虏茅虏脦脢媒脢脟路帽脭脷0潞脥1脰庐录盲
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
        # self.container            = [rtree.index.Index(), rtree.index.Index(),rtree.index.Index()]
        self.lineDict             = dict()
    def __calSize(self):
        len = 0
        for lineList in self.lines:
            len += len(lineList)
        return len
    # def queryIntersect(self, line):
    #     itsList = []
    #     minPoint = Point(min(line.start.x, line.end.x), min(line.start.y, line.end.y), min(line.start.z, line.end.z))
    #     maxPoint = Point(max(line.start.x, line.end.x), max(line.start.y, line.end.y), max(line.start.z, line.end.z))

    #     for item in self.container:
    #         intersecting_segments = item.intersection((minPoint.x, minPoint.y, maxPoint.x, maxPoint.y))
    #         for sid in intersecting_segments:
    #             curLine = self.lineDict[sid]
                 
    #             if curLine.isIntersect(line) != None:
    #                 itsList.append(curLine)
    #     return itsList
    def queryIntersectByAll(self, line):
        itsList = []
        for item in self.lines:
            for curLine in item:                 
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
        if(int(axis.value) < int(AxisEnum.AXIS_X.value) or int(axis.value) > (AxisEnum.AXIS_Z.value)):
            print(u"Error: invalid index:{}".format(axis))
            return

        self.lines[axis.value].append(line)
        self.lineDict[line.id] = line
        
        # #add to RTree
        # minPoint = Point(min(line.start.x, line.end.x), min(line.start.y, line.end.y), min(line.start.z, line.end.z))
        # maxPoint = Point(max(line.start.x, line.end.x), max(line.start.y, line.end.y), max(line.start.z, line.end.z))
        # self.container[axis.value].insert(int(line.id), (minPoint.x, minPoint.y, maxPoint.x, maxPoint.y), obj=line)

class Path(object):
    def __init__(self):
        self.name = ""
        self.segs = []
        self.vias = []
        self.shapes = []
        self.similarity = 0.0
        self.w = 0
        self.s = 0
    def getName(self):
        return self.name
    def setName(self, nm):
        self.name = nm
    def setWidth(self, w):
        self.w = w
    def setSpace(self, s):
        self.s = s
    def setSimilarity(self, sim):
        self.similarity = sim

    def addSeg(self, seg):
        self.segs.append(seg)
    def setCenterLine(self, centerLine):
        for i in range(len(centerLine)-1):
            p1 = centerLine[i] 
            p2 = centerLine[i+1]
            if p1[2] != p2[2]:
                self.vias.append([p1[0], p1[1], p1[2], p2[2]])
                continue
            pt1 =  Point(p1[0], p1[1], p1[2])
            pt2 =  Point(p2[0], p2[1], p2[2])
            self.segs.append(Line(pt1, pt2))

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
            seg.printLines()
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
    def formatToDict(self):
        resultDict = {str(self.name): None}

        contentDict = {}
        pathList = []
        for seg in self.segs:
            segDict = {}
            p1 = seg.start
            p2 = seg.end
            w  = self.w
            s  = self.s

            segDict["layer"] = int(p1.z)
            segDict["poly"]  = [[p1.x, p1.y],[p2.x, p2.y]]
            segDict["width"] = w
            pathList.append(segDict)
        contentDict["path"] = pathList
        contentDict["via"] = None

        resultDict[str(self.name)] = contentDict
        return contentDict
        
class ListQueue:
    def __init__(self):
        self.queue = []

    def is_empty(self):
        return len(self.queue) == 0

    def append(self, item):
        self.queue.append(item)

    def popleft(self):
        if self.is_empty():
            raise IndexError("dequeue from an empty queue")
        return self.queue.pop(0)

    def size(self):
        return len(self.queue)

 
colors = [u"red", u"blue", u"magenta", u"green", u"deepskyblue", u"gold", u"lime"]
class Grid3D(object):
    def __init__(self, w, h, l, r=1, oft=[0,0,0]):
        self.w = int(w/r)  # col
        self.h = int(h/r)  # row
        self.l = l  # layer
        self.r = r  # resolution
        self.offset = oft
        self.s0     = self.w*2
        self.delta  = 5#min(self.h, self.w)*0.1
        self.grid      = [] # np.zeros((w, h, l), dtype=bool)   # 脫脙脫脷麓忙麓垄脮录脫脙脳麓脤卢[z][x][y]
        self.smrMap    = [] # np.zeros((w, h, l), dtype=float)  # save similarity map[z][x][y]
        self.obstacles = [] # list([[(x1,y1,z1),(x2,y2,z2)...],[obs2],...])
        self.expand    = 0
        self.__initGridMap()
        self.__initSmrMap()

    def __initGridMap(self):
        for z in range(self.l):
            xLst = []
            for x in range(self.w):
                yLst = []
                for y in range(self.h):
                    yLst.append(False)
                xLst.append(yLst)
            self.grid.append(xLst)
   
    def __initSmrMap(self):
        for z in range(self.l):
            xLst = []
            for x in range(self.w):
                yLst = []
                for y in range(self.h):
                    yLst.append(float('inf'))
                xLst.append(yLst)
            self.smrMap.append(xLst)
         
    def isValid(self, point):
        if (0 <= point[0] < self.w and 0 <= point[1] < self.h and 0 <= point[2] < self.l):
            return True
        else:
            return False
    
    def _rasterization(self,  polygon, e=0):
        points=[]
        X = [ item[0] for item in polygon]
        Y = [ item[1] for item in polygon]
        Z = [ item[2] for item in polygon]  
        min_x = min(X) #- e
        min_y = min(Y) #- e
        max_x = max(X) #+ e
        max_y = max(Y) #+ e
        layer =  min(Z)
        minPt = self.point2Gpoint((min_x,min_y,layer))
        maxPt = self.point2Gpoint((max_x,max_y,layer))
        layer = minPt[2]
        # print("minPt:{0}, maxPt:{1}".format(minPt, maxPt))
        for x in range(minPt[0], maxPt[0]):
            for y in range(minPt[1], maxPt[1]):
                points.append((x,y,layer))
        return points
    def set_obstacle(self, polygons, e = 0):
        self.obstacles = polygons
        self.expand    = e
        for polygon in polygons:
            X = [ item[0] for item in polygon]
            Y = [ item[1] for item in polygon]
            Z = [ item[2] for item in polygon]  
            min_x = min(X) - e
            min_y = min(Y) - e
            max_x = max(X) + e
            max_y = max(Y) + e
            layer         =  min(Z)
            minPt = self.point2Gpoint((min_x,min_y,layer))
            maxPt = self.point2Gpoint((max_x,max_y,layer))
            layer = minPt[2]
            # print("minPt:{0}, maxPt:{1}".format(minPt, maxPt))

            for x in range(minPt[0], maxPt[0]):
                for y in range(minPt[1], maxPt[1]):
                    self.grid[layer][x][y] = True   
                    self.smrMap[layer][x][y]  = 0 
        # for polygon in polygons:
        #     points = self._rasterization(polygon, e)
        #     # print(points
        #     for pt in points:
        #         x = pt[0]
        #         y = pt[1]
        #         layer = pt[2]
        #         # self.grid[x, y, layer]   = True  # 脭脷露脭脫娄虏茫脡猫脰脙脮脧掳颅
        #         # self.smrMap[x, y, layer] = 0
        #         self.grid[layer][x][y]    = True  # 脭脷露脭脫娄虏茫脡猫脰脙脮脧掳颅
        #         self.smrMap[layer][x][y]  = 0
         
    def getSmrValue(self, pt):
        # return self.smrMap[pt]
         return self.smrMap[pt[2]][pt[0]][pt[1]]
    def is_occupied(self, pt):
        if (0 <= pt[0] < self.w)  and  (0 <= pt[1] < self.h)  and (0 <= pt[2] < self.l):
            return self.grid[pt[2]][pt[0]][pt[1]]
        #    return self.grid[pt[0], pt[1], pt[2]]
        else:
            return False

    def get_neighbors(self, x, y, z, diagonal=False):
        u"""禄帽脠隆脥酶赂帽碌脛 4 脕脷脫貌禄貌 8 脕脷脫貌碌脛露楼碌茫."""
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        if diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        neighbors = []
        for dx, dy in directions:
            nx, ny, nz = x + dx, y + dy, z
            if 0 <= nx < self.w and 0 <= ny < self.h and (self.is_occupied((nx, ny, nz)) == False):
                neighbors.append((nx, ny, nz))

        u""" layer change """
        layers = [-1, 1]
        for dz in layers:
            nz = z + dz
            if nz < 0 or nz >= self.l: 
                continue

            if(self.is_occupied((x, y, nz)) == False):
                # pass
                neighbors.append((x,y, nz))
        # print("neighbor: ", neighbors)
        return neighbors
     
    def setNetTemplate(self, points, lyr, width):
        polygons = self.getPolyShape(points, lyr, width)
        return polygons
    def updateSmrForDisMatchSeg(self, misSegPts, penalty):
        penaltyPts = []
        for i in range(len(misSegPts)-1):
            start = misSegPts[i]
            end   = misSegPts[i+1]
            if(start[0] == end[0] and start[1] != end[1]):
                s = min(start[1], end[1])
                m = max(start[1], end[1])
                yLst = [y  for y in range(s, m+1, 1)]
                for y in yLst:
                    penaltyPts.append((start[0], y, start[2]))
            elif(start[1] == end[1] and start[0] != end[0]):
                s = min(start[0], end[0])
                m = max(start[0], end[0])
                xLst = [x  for x in range(s, m+1, 1)]
                for x in xLst:
                    penaltyPts.append((x, start[1], start[2]))
            else:
                print(u"Not support for angle segment")
                return 
        
        penaltyPts = set(penaltyPts)
        for pt in penaltyPts:
            # self.smrMap[pt] = max((self.smrMap[pt] - penalty), 0)
            # self.smrMap[pt[2]][pt[0]][pt[1]] = max((self.smrMap[pt[2]][pt[0]][pt[1]] - penalty), 0)
            self.smrMap[pt[2]][pt[0]][pt[1]] = min((self.smrMap[pt[2]][pt[0]][pt[1]] + penalty), float('inf'))
    def getInterSectObstacles(self, centerLine):
        # obstacle to rectangle line set
        obsLineSets = []
        for obs in self.obstacles:
            min_x = min([pt[0] for pt in obs]) - self.expand
            min_y = min([pt[1] for pt in obs]) - self.expand
            max_x = max([pt[0] for pt in obs]) + self.expand
            max_y = max([pt[1] for pt in obs]) + self.expand
            z     = obs[0][2]

            line1 = Line(Point(min_x, min_y, z), Point(min_x, max_y, z))
            line2 = Line(Point(min_x, max_y, z), Point(max_x, max_y, z))
            line3 = Line(Point(max_x, max_y, z), Point(max_x, min_y, z))
            line4 = Line(Point(min_x, min_y, z), Point(max_x, min_y, z))
            obsLineSets.append([line1,line2, line3, line4])
        #centerLine to line set
        centerLineSet = []
        for i in range(len(centerLine)-1):
            start = centerLine[i]
            end   = centerLine[i+1]
            if (start[0] == end[0] and start[1] == end[1] and start[2] != end[2]):
                continue


            tmpLine = Line(Point(start[0],start[1],start[2]), Point(end[0],end[1],end[2]))
            centerLineSet.append(tmpLine)
        
        # get intersection obstacles
        interSectObs = []
        for i in range(len(centerLineSet)):
            cLine = centerLineSet[i]
            for obsSet in obsLineSets:              
                for oLine in obsSet:
                    pt =  cLine.isIntersect(oLine)
                    if pt != None:
                        interSectObs.append(obsSet)
                        break
        print("interSectObstalce: ", interSectObs)
        return interSectObs

    def extractLineToPointSets(self, line):
        startPoints = []
        start = (line.start.x, line.start.y, line.start.z)
        end   = (line.end.x,   line.end.y,  line.end.z)
        if(start[0] == end[0] and start[1] != end[1]):
            s = min(start[1], end[1])
            m = max(start[1], end[1])
            yLst = [y  for y in range(s, m+1, 1)]
            for y in yLst:
                startPoints.append((start[0], y, start[2]))
        elif(start[1] == end[1] and start[0] != end[0]):
            s = min(start[0], end[0])
            m = max(start[0], end[0])
            xLst = [x  for x in range(s, m+1, 1)]
            for x in xLst:
                startPoints.append((x, start[1], start[2]))
        else:
            print(u"Not support for any angle segment")
        
        return startPoints      


        # judge line intersect with obstacle's line set
    def calSmrWithBFS(self, polygons, centerLine= False):
        print(u"size:", (self.w, self.h, self.l))
        # init template as maximum
        startPoints = []
        if(centerLine == True):
            # extract to line Set
            for i in  range(len(polygons)-1):
                start = self.point2Gpoint(polygons[i])
                end   = self.point2Gpoint(polygons[i+1])
                if(start[0] == end[0] and start[1]== end[1] and start[2]!=end[2]):
                    continue
                line = Line(Point(start[0], start[1], start[2]), Point(end[0], end[1], end[2]))
                sPt = self.extractLineToPointSets(line) 
                for pt in sPt:
                    startPoints.append(pt)
                    pass
                

            #extract interSectObs    
            obsLineSet = self.getInterSectObstacles(polygons)
            for obs in obsLineSet:
                print("obs",obs)
                for oLine in obs:
                    sPt = self.extractLineToPointSets(oLine)
                    for pt in sPt:
                        pt = self.point2Gpoint(pt)
                        startPoints.append((pt[0],pt[1],pt[2])) 

        else:
            for polygon in polygons:
                tPoints = self._rasterization(polygon)
                startPoints.extend(tPoints)

        
        # unique
        # print("startPoints:", startPoints)        
        startPoints = set(startPoints)
        print("startPoints:", startPoints)
        
        #visited sets
        visited = set()  #  
        directions = [(-1, 0, 0), (1, 0,0), (0, -1,0), (0, 1,0), (0, 0, -1), (0, 0,1)]

        #initialized initial queue
        queue = ListQueue()  
        for pt in startPoints:
            queue.append(pt)
            self.smrMap[pt[2]][pt[0]][pt[1]] = 0 #self.s0
        while queue.size() != 0:
            current = queue.popleft()
            smrVal = 0.0
            neighbors = []
            for direction in directions:
                nx = (current[0] + direction[0])  
                if nx <0 or nx >= self.w :
                    continue

                ny = (current[1] + direction[1])  
                if ny <0 or ny >= self.h :
                    continue

                nz = (current[2] + direction[2])
                if nz <0 or nz >= self.l :
                    continue
                
                neighbor = (nx, ny, nz)
                if self.grid[nz][nx][ny] == True:
                    continue
                neighbors.append(neighbor)
            
            #init map
            for neighbor in neighbors:
                if ( 0 <= neighbor[0] < self.w and 0 <= neighbor[1] < self.h and 
                     neighbor not in visited  and  self.grid[neighbor[2]][neighbor[0]][neighbor[1]] == False):
                    sExist = self.smrMap[neighbor[2]][neighbor[0]][neighbor[1]]
                    if neighbor not in startPoints:   
                        queue.append(neighbor)
                        # smrVal = self.smrMap[current[2]][current[0]][current[1]]-  self.delta - neighbor[2]*self.s0*0.5
                        smrVal = self.smrMap[current[2]][current[0]][current[1]] +  self.delta + neighbor[2]*0.5  #*self.s0
                        # if(smrVal < 0):
                        #     smrVal = 0
                    else:
                        # smrVal =  self.s0
                        smrVal =  0
                    
                    # if(smrVal == 0):
                    #     break
                    visited.add(neighbor)
                    # self.smrMap[neighbor] = smrVal #max(sExist, smrVal)
                    self.smrMap[neighbor[2]][neighbor[0]][neighbor[1]] = smrVal#max(sExist, smrVal)
    
    def generateParallelSeg(self,cline, distance):
        x1 = 0
        x2 = 0
        y1 = 0
        y2 = 0
        start = cline[0]
        end   = cline[1]
        dx = float(end[0] - start[0])
        dy = float(end[1] - start[1])
        length = math.sqrt(dx * dx + dy * dy)
        perpX  = -float(dy) / length
        perpY  =  float(dx) / length
        x1 = start[0] + int(distance * perpX)
        y1 = start[1] + int(distance * perpY)
        x2 = end[0]   + int(distance * perpX)
        y2 = end[1]   + int(distance * perpY)
        return [(int(x1), int(y1)), (int(x2), int(y2))]
    
    # get dis-match segments
    def getUnitDirectionVector(self, startPt, endPt):
        u"""
        #get unit vector
        start: (x, y, z)
        end  : (x, y, z)
        """
        norm    = math.sqrt((endPt[0]-startPt[0])**2 + (endPt[1]-startPt[1])**2 + (endPt[2]-startPt[2])**2)
        unitVec = ((endPt[0]-startPt[0])/norm, (endPt[1]-startPt[1])/norm, (endPt[2]-startPt[2])/norm)
        return unitVec
    def getDisMatchSegmentRange(self, tpCtrLine, resCtrLine):
        # transform to unit vector
        uTpVec = []
        for i in range(len(tpCtrLine)-1):
            startPt = tpCtrLine[i]
            endPt   = tpCtrLine[i+1]
            unitVec = self.getUnitDirectionVector(startPt, endPt)
            uTpVec.append(unitVec)
        uRsVec = []
        for i in range(len(resCtrLine)-1):
            startPt = resCtrLine[i]
            endPt   = resCtrLine[i+1]
            unitVec = self.getUnitDirectionVector(startPt, endPt)
            uRsVec.append(unitVec)
        
        # match by both size: resCtrLine start point the same as tpCtrLine
        misStart = int(-1)
        misEnd   = int(-1)
        # 1: find the first un-match segment index
        for i in range(len(uRsVec)):
            if not (i < len(uTpVec)):
                misStart = i
                break
            if(uTpVec[i] == uRsVec[i]):
                continue
            else:
                misStart= i
                break

        # 2
        uTpVec_reverse = uTpVec[::-1]
        uRsVec_reverse = uRsVec[::-1]
        for i in range(len(uRsVec_reverse)):
            if not (i < len(uTpVec_reverse)):
                misEnd = len(uRsVec_reverse)- i
                break
            
            if(uRsVec_reverse[i] == uTpVec_reverse[i]):
                continue
            else:
                misEnd= len(uRsVec_reverse)- i
                break

        segPts = []
        if(misStart == -1 and misEnd == -1):
            return segPts
        else:
            if(misStart > misEnd):
                misStart, misEnd = misEnd, misStart
            elif (misStart ==misEnd):
                misEnd = min(misEnd + 1 , len(uRsVec_reverse)-1)
            for i in range(misStart, misEnd+1):
                segPts.append(resCtrLine[i])
        # print("segPts: ", segPts)
        return segPts
    def calculateMatchValue(self, tpCtrLine, resCtrLine):
        segPts = self.getDisMatchSegmentRange(tpCtrLine, resCtrLine)        
        v = (len(resCtrLine) - len(segPts))/len(resCtrLine)
        return v

    #center line [(1,1), (10, 1), (10, 10)]
    def getPolyShape(self, centerLine, lyr, width):
        polygons = []
        for  i in range(len(centerLine)-1):
            first =  centerLine[i]  
            second = centerLine[i+1] 
            cline = [first, second]

            d =  float(width / 2.0)
            polyPts  = []
            seg1 = self.generateParallelSeg(cline,  d)  
            seg2 = self.generateParallelSeg(cline, -d)  
            polyPts.append([seg1[0][0], seg1[0][1], lyr])  
            polyPts.append([seg2[0][0], seg2[0][1], lyr])  
            polyPts.append([seg2[1][0], seg2[1][1], lyr])  
            polyPts.append([seg1[1][0], seg1[1][1], lyr])  

            if width == 0:
                polyPts = [[first[0], first[1], lyr], [second[0], second[1], lyr]]
            polygons.append(polyPts)
        return polygons

    def isStraitLine(self, p1, p2, p3):
        crossProductX = (p2[1] - p1[1]) * (p3[2] - p1[2]) - (p2[2] - p1[2]) * (p3[1] - p1[1])
        crossProductY = (p2[2] - p1[2]) * (p3[0] - p1[0]) - (p2[0] - p1[0]) * (p3[2] - p1[2])
        crossProductZ = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
        return (crossProductX == 0 and crossProductY == 0 and crossProductZ == 0)

    def simplePointSet(self, res):
        path= []
        invalidCoord = (-1, -1, -1)
        frontCoord =  (-1, -1, -1)
        midCoord   =  (-1, -1, -1)
        backCoord  =  (-1, -1, -1)
        for i in range(len(res)):
            frontCoord = res[i]
            if i == (len(res) - 1):   
                if (frontCoord!= invalidCoord and midCoord != invalidCoord and backCoord != invalidCoord):
                    if (self.isStraitLine(frontCoord, midCoord, backCoord)):
                        path.append(backCoord)
                        path.append(frontCoord)
                        continue
                    else:
                        path.append(backCoord)
                        path.append(midCoord)
                        path.append(frontCoord)
                        continue
                elif (frontCoord != invalidCoord and  midCoord!= invalidCoord  and  backCoord == invalidCoord):
                    path.append(midCoord)
                    path.append(frontCoord)
                    continue
            if (frontCoord!= invalidCoord and midCoord != invalidCoord and backCoord != invalidCoord):
                if (self.isStraitLine(frontCoord, midCoord, backCoord)):
                    midCoord = frontCoord
                    continue

            if (backCoord != invalidCoord):
                path.append(backCoord)
            backCoord = midCoord
            midCoord = frontCoord
        return path[::-1]
    
    def point2Gpoint(self, pt):
        x = int((pt[0]-self.offset[0])/self.r)
        y = int((pt[1]-self.offset[1])/self.r)
        z = int(pt[2]-1)  # layer start with 1

        if x < 0:
            x = 0
        if y <0:
            y = 0
        if z <0:
            z = 0

        if (x >= self.w):
            x = (self.w - 1)
        if (y >= self.h):
            y = (self.h - 1)
        if (z>=self.l):
            z = (self.l -1)
        tp = [x , y, z]
        return tp
     
    def gPoint2point(self, pt):
        x = pt[0] * self.r + self.offset[0]
        y = pt[1] * self.r + self.offset[1]
        z = pt[2] + 1 # layer start with 1
        tp = [x,y,z] 
        return tp
    def drawShapelyPolygon(self,fig, ax, polygons, cr = u"", ap = 1, fl = True):
        # fig, ax = plt.subplots(figsize=(w, h)) 
        ax.set_aspect(u'equal')
        ax.set_title(u'Polygon Visualization')
        for polygon in polygons:
            pts = []
            lyr = polygon[0][2]
            for (x,y, z) in polygon:
                pt = self.point2Gpoint([x,y,z])
                pts.append([pt[0],pt[1]])
            if cr == u"":
                cr = colors[lyr]
            p = MatplotlibPolygon(xy=pts, closed=True, fill=fl, facecolor=cr,alpha=ap, edgecolor=cr, linewidth=2)
            ax.add_patch(p)
        # plt.show()
    def drawSmrMap(self):
        tmpSmrMap = np.zeros((self.w, self.h, self.l), dtype=float)
        for x in range(self.w):
            for y in range(self.h):
                for z in range(self.l):
                    tmpSmrMap[x][y][z] = self.smrMap[z][x][y]
        rotated_about_z = np.transpose(tmpSmrMap, (1, 0, 2))
        plt.figure() 
        for z in range(self.l):
            plt.subplot(self.l, 1, z + 1)
            plt.imshow(rotated_about_z[:, :, z], cmap=u'viridis', interpolation=u'nearest', origin=u'lower')
            plt.title('Layer {}'.format(z + 1))
            plt.colorbar(label=u'smrMap')
            plt.xlabel(u'X')
            plt.ylabel(u'Y')
        plt.tight_layout()
        # plt.show()
if __name__ == u'__main__':    
    s_start = (5, 5, 0)
    s_goal  = (20, 5, 0)
    tpZ     = [(5,5, 0),(5,40, 0),(20,40, 0), (20, 5, 0)]
    ob      = [[20, 15,0], [30, 15,0], [30, 20,0], [20, 20,0]]  
    
    centerPts = [tpZ]
    obstacles = [ob]
    w, h, l,r = 50, 50, 3,1
    rtGraph = Grid3D(w, h, l, r)

    #Initialize layout obstacle to routing graph
    path_obs=[]
    if(len(centerPts) >1):
        for i in range(1, len(centerPts)):
            path_obs     = rtGraph.setNetTemplate(centerPts[i], 0, 2)  #show z template obstacle
            obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles) 
     
 
 