# coding=utf-8
from __future__ import division
from __future__ import absolute_import
import numpy as np

import matplotlib.pyplot as plt

from matplotlib.patches import Polygon as MatplotlibPolygon
 

import math

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

# 定义颜色
colors = [u"red", u"blue", u"magenta", u"green", u"deepskyblue", u"gold", u"lime"]
class Grid3D(object):
    def __init__(self, w, h, l, r):
        self.w = w  # 长度
        self.h = h  # 高度
        self.l = l  # 层数
        self.r = r  # resolution
        self.s0  = self.w*2
        self.delta = 1
        self.grid   = []# np.zeros((w, h, l), dtype=bool)  # 用于存储占用状态[z][x][y]
        self.smrMap = []#np.zeros((w, h, l), dtype=float)  # save similarity map[z][x][y]
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
                    yLst.append(0.0)
                xLst.append(yLst)
            self.smrMap.append(xLst)
         
    def isValid(self, point):
        if (0 <= point[0] < self.w and 0 <= point[1] < self.h and 0 <= point[2] < self.l):
            return True
        else:
            return False
    
    def _rasterization(self,  polygon):
        points=[]
        X = [ item[0] for item in polygon]
        Y = [ item[1] for item in polygon]
        Z = [ item[2] for item in polygon]
        # 获取多边形的包络矩形
        min_x, min_y  =  min(X), min(Y)
        max_x, max_y  =  max(X), max(Y)
        layer         =  min(Z)
   
        # spPoly = Polygon(polygon)
        # print("spPoly", spPoly)
        for x in range(int(min_x), int(max_x) +1):
            for y in range(int(min_y), int(max_y) +1):
                points.append((x,y,layer))
                # if spPoly.intersects(Point(x,y)):
                #     points.append((x,y,layer))
        return points
    def set_obstacle(self, polygons):
        for poly in polygons:
            points = self._rasterization(poly)
            for pt in points:
                x = pt[0]
                y = pt[1]
                layer = pt[2]
                # self.grid[x, y, layer]   = True  # 在对应层设置障碍
                # self.smrMap[x, y, layer] = 0
                self.grid[layer][x][y]    = True  # 在对应层设置障碍
                self.smrMap[layer][x][y]  = 0
         
    def getSmrValue(self, pt):
        # return self.smrMap[pt]
         return self.smrMap[pt[2]][pt[0]][pt[1]]
    def is_occupied(self, pt):
        u"""判断网格是否被占用."""
        # print("pt: ", pt)
        # print("gridStatus: " , self.grid[pt[0], pt[1], pt[2]])
        if (0 <= pt[0] < self.w)  and  (0 <= pt[1] < self.h)  and (0 <= pt[2] < self.l):
            return self.grid[pt[2]][pt[0]][pt[1]]
        #    return self.grid[pt[0], pt[1], pt[2]]
        else:
            return False

    def get_neighbors(self, x, y, z, diagonal=False):
        u"""获取网格的 4 邻域或 8 邻域的顶点."""
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

    def draw_grid(self):
        u"""绘制 3D 网格地图的视图."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection=u'3d')
        
        # 绘制占用的格子MatplotlibPolygon
        occupied = np.argwhere(self.grid)
        for x, y, z in occupied:
            ax.bar3d(x, y, z, 1, 1, 1, color=colors[z], alpha=0.5)

        ax.set_xlabel(u'X')
        ax.set_ylabel(u'Y')
        ax.set_zlabel(u'Z')
        ax.set_title(u'3D Grid Map')
        plt.show()
     
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
            self.smrMap[pt[2]][pt[0]][pt[1]] = max((self.smrMap[pt[2]][pt[0]][pt[1]] - penalty), 0)
                     
    def calSmrWithBFS(self, polygons, centerLine= False):
        print(u"size:", (self.w, self.h, self.l))
        # init template as maximum
        startPoints = []
        if(centerLine == True):
            for i in  range(len(polygons)-1):
                start = polygons[i]
                end   = polygons[i+1]
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
                    print(u"Not support for angle segment")
                    return
        else:
            for polygon in polygons:
                tPoints = self._rasterization(polygon)
                startPoints.extend(tPoints)


        # unique
        # print("startPoints:", startPoints)        
        startPoints = set(startPoints)
        
        #visited sets
        visited = set()  # 记录访问过的节点
        directions = [(-1, 0, 0), (1, 0,0), (0, -1,0), (0, 1,0), (0, 0, -1), (0, 0,1)]

        #initialized initial queue
        queue = ListQueue()  # 使用队列存储待访问的节点
        for pt in startPoints:
            queue.append(pt)
            # self.smrMap[pt] = self.s0
            self.smrMap[pt[2]][pt[0]][pt[1]] = self.s0
        while queue.size() != 0:
            current = queue.popleft()
 
            # 遍历相邻节点
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
                neighbors.append(neighbor)
            
            #init map
            for neighbor in neighbors:
                #Field calculate
              
                # 检查邻居是否在网格内且未被访问且不是障碍
                # if ( 0 <= neighbor[0] < self.h and 0 <= neighbor[1] < self.w and 
                #      neighbor not in visited  and  self.grid[neighbor[0],neighbor[1],neighbor[2]] == False):
                if ( 0 <= neighbor[0] < self.h and 0 <= neighbor[1] < self.w and 
                     neighbor not in visited  and  self.grid[neighbor[2]][neighbor[0]][neighbor[1]] == False):
                    sExist = self.smrMap[neighbor[2]][neighbor[0]][neighbor[1]]
                    if neighbor not in startPoints:   
                        queue.append(neighbor)
                        smrVal = self.smrMap[current[2]][current[0]] [current[1]]-  self.delta - neighbor[2]*self.s0*0.5
                        # smrVal = self.smrMap[current] -  self.delta - neighbor[2]*self.s0*0.5
                        if(smrVal < 0):
                            smrVal = 0
                    else:
                        smrVal =  self.s0
                    
                    # if(smrVal == 0):
                    #     break
                    visited.add(neighbor)
                    # self.smrMap[neighbor] = smrVal #max(sExist, smrVal)
                    self.smrMap[neighbor[2]][neighbor[0]][neighbor[1]] = smrVal
 
    def drawShapelyPolygon(self,fig, ax, polygons, cr = u"", ap = 1, fl = True):
        # fig, ax = plt.subplots(figsize=(w, h)) 
        ax.set_aspect(u'equal')
        ax.set_title(u'Polygon Visualization')
        for polygon in polygons:
            pts = []
            lyr = polygon[0][2]
            for (x,y, z) in polygon:
                pts.append([x,y])
            if cr == u"":
                cr = colors[lyr]
            p = MatplotlibPolygon(xy=pts, closed=True, fill=fl, facecolor=cr,alpha=ap, edgecolor=cr, linewidth=2)
            ax.add_patch(p)
        # plt.show()
    
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
        # print(">>tp unit vector:", uTpVec)
        # print(">>tp tpCtrLine:", tpCtrLine)
        # print("<<res unit vector:", uRsVec)
        # print("<<res resCtrLine:", resCtrLine)
        
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
        # print("missStar:{}, missEnd:{}".format(misStart, misEnd))

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
        # print("centerLine: ", len(centerLine))
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
            if i == (len(res) - 1):  #last element
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
                # when there is only two points (one segment) in this path
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
    #show design
    fig, ax = plt.subplots(figsize=(w, h)) 
    ax.set_xlim(0, w + 0.1)
    ax.set_ylim(0, h + 0.1)
    rtGraph.drawShapelyPolygon(fig, ax, obstacles,  u"black", 1)
    templateShow   = rtGraph.setNetTemplate(centerPts[0], 0, 2)  #tempalte   Z
    rtGraph.drawShapelyPolygon(fig, ax, templateShow,  u"yellow", 0.4)
    plt.show()
 
 