
# coding=utf-8
from __future__ import division
from __future__ import absolute_import
import os
import sys
import math
import copy
# import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from smrRouteGraph import Grid3D, Point, Line, LineContainer, Path
from smrRouteGraph import TopologyEnum, AxisEnum
import matplotlib.pyplot as plt                  
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
        return tp    
    def __calSimilarity(self, path, template):
        simValue = 0.0
        if(path == None): 
            return simValue
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
        
        if(len(orgTp) > len(resTp)):
            simValue = count/len(orgTp)
        else:
            simValue = count/len(resTp)
        print(u"Similarity: {0}\n".format(simValue))
        return simValue    
    def __beExtendInAxis(self, ePt, dir):
        return (ePt in self.allPointsDT and self.allPointsDT[ePt][dir.value] == 1)  
    def __duplicateProcess(self, ln):
        if ln !=None:
            for point in ln.pointSet:
                state = [0,0,0]
                if point not in self.allPointsDT:
                    state[ln.axisDirect.value] =  1
                    self.allPointsDT[point] = state
                else:
                    self.allPointsDT[point][ln.axisDirect.value] = 1
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
            if(start.x== end.x and start.y == end.y and start.z != end.z):
                continue
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
            # lines = targetContainer.queryIntersect(line1)
            lines = targetContainer.queryIntersectByAll(line1)
            if(len(lines) == 0):
                continue
            else:
                for ll in lines:
                    path = self.getPath(line1, ll)
                    cdtPaths.append(path)
        return cdtPaths
 
    
    def findPath(self, s, e, tpS = TopologyEnum.ANY, tpT = TopologyEnum.ANY, ):
        self.allPointsDT= {}
        #tmp
        visitSource = []
        visitTarget = []

        start = Point(s[0], s[1], s[2])
        goal  = Point(e[0], e[1], e[2])
        # invalid
        if self.rtGraph.is_occupied((start.x, start.y, start.z)):
            print(u"Start is occupied!")
            return []
        if self.rtGraph.is_occupied((goal.x, goal.y, goal.z)):
            print(u"Target is occupied")
            return []
        
        #create s/t line
        startLines     =  self.createLineAll(start, tpS)
        self.startSet.setLines(startLines)
        # for l in startLines:
        #     l.draw(u'red')
        goalLines   = self.createLineAll(goal, tpT)
        self.targetSet.setLines(goalLines)
        # for l in goalLines:
        #     l.draw(u'green') 
        
        path =  self.getBestLengthPath(startLines, self.targetSet)
        while len(startLines) !=0 and len(goalLines)!=0 and path ==None:
            #expand S
            expandList = []
            for subLine in startLines:
                e = self.expandLine(subLine, self.startSet)
                e = sorted(e, key= lambda line: self.__sortLambda(line, goal))
                expandList.extend(e)
            self.startSet.setLines(expandList)
            visitSource.append(expandList)   
            path  = self.getBestLengthPath(expandList, self.targetSet)
            if (path != None):
                self.path = path
                print(u"Find path success from source !")
                break
            startLines =   expandList

            #expand T
            expandListT = []
            for subLine in goalLines:
                e = self.expandLine(subLine, self.targetSet)
                e = sorted(e, key= lambda line: self.__sortLambda(line, start))
                expandListT.extend(e) 
            self.targetSet.setLines(expandListT)
            visitTarget.append(expandListT)
            path  = self.getBestLengthPath(expandListT, self.startSet)
            if (path != None):
                self.path = path
                print(u"Find path success from target !")
                break
            goalLines = expandListT
        
        # self.showAlgoResult(visitSource, visitTarget, path)
        
    def dfs(self, traversed, curLine,  start, goal, sTopoList, template, level):
        curTopoList = copy.deepcopy(sTopoList)  
        tp = TopologyEnum.ANY
        # print(u"curLevel:{0}, len(topList):{1}".format(level, len(curTopoList))
        if len(curTopoList) != 0:
            tp = curTopoList.pop(0)    
        # print(u"currentTopo:{0}".format(tp)
        if curLine not in traversed:
            if self.path != None:
                return
            #find Path
            path  = self.getSimilarityPath([curLine], template, self.targetSet)
            if path != None and len(curTopoList)==0:
                self.path = path
                print(u"find Path")
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
        self.allPointsDT= {}
        #calculate topology
        print("template:", template)
        topo = self.__topologyExtract(template)
        
        #tmp
        start = Point(s[0], s[1], s[2])
        goal  = Point(e[0], e[1], e[2])

        # Invalid Judgement
        if self.rtGraph.is_occupied((start.x, start.y, start.z)):
            print(u"Start is occupied!")
            return []
        if self.rtGraph.is_occupied((goal.x, goal.y, goal.z)):
            print(u"Target is occupied")
            return []
        
        #topo proccess
        mirroTopo = self.__getMirrorTopo(topo)
        print(u"Topo:", topo)
        print(u"mirroTopo:", mirroTopo)
        

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
            print(u"Find Path failed with template! start find without template!")
            self.findPath(s,e, topo[0], tTopoList[0])
            if( self.path != None):
                self.path.similarity = self.__calSimilarity(self.path, sTmpList) 
             
        self.showAlgoResult([traverseList], [goalLines], self.path)
        if self.path !=None:
            for seg in self.path.segs:
                s = self.rtGraph.gPoint2point([seg.start.x, seg.start.y, seg.start.z])
                t = self.rtGraph.gPoint2point([seg.end.x, seg.end.y, seg.end.z])
                seg.start = Point(s[0], s[1], s[2])
                seg.end   = Point(t[0], t[1], t[2])
        
        return self.path

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
            print(u"Path similarity: ", path.similarity)
            path.draw()

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
    rtGraph =  Grid3D(w, h, l, r)

    #Initialize layout obstacle to routing graph
    path_obs=[]
    if(len(centerPts) >1):
        for i in range(1, len(centerPts)):
            path_obs     = rtGraph.setNetTemplate(centerPts[i], 0, 2)  #show z template obstacle
            obstacles.extend(path_obs)
    rtGraph.set_obstacle(obstacles) 
    #show design
    # fig, ax = plt.subplots(figsize=(w, h)) 
    # ax.set_xlim(0, w + 0.1)
    # ax.set_ylim(0, h + 0.1)
    # rtGraph.drawShapelyPolygon(fig, ax, obstacles,  u"black", 1)
    # templateShow   = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    # rtGraph.drawShapelyPolygon(fig, ax, templateShow,  u"yellow", 0.4)

    #Find current net's path with template
    mkt =  MikamiTauchi(rtGraph, multilayer)
    mkt.findPathWithTemplate(s_start, s_goal, centerPts[0])

    print(u"Routing finish!\n")
    
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