import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import Polygon as MatplotlibPolygon
from shapely.geometry import LineString
# from shapely.geometry.polygon import Polygon
from shapely.geometry import Point, Polygon
from collections import deque
import math



# 定义颜色
colors = ["red", "blue", "magenta", "green", "deepskyblue", "gold", "lime"]
class Grid3D:
    def __init__(self, w, h, l, r):
        self.w = w  # 长度
        self.h = h  # 高度
        self.l = l  # 层数
        self.r = r  # resolution
        self.s0    = self.w*2
        self.delta = 1
        self.grid   = np.zeros((w, h, l), dtype=bool)  # 用于存储占用状态
        self.smrMap = np.zeros((w, h, l), dtype=float)  # save similarity map
    def _rasterization(self,  polygon):
        points=[]
        X = [ item[0] for item in polygon]
        Y = [ item[1] for item in polygon]
        Z = [ item[2] for item in polygon]
        # 获取多边形的包络矩形
        min_x, min_y  =  min(X), min(Y)
        max_x, max_y  =  max(X), max(Y)
        layer         =  min(Z)
        vtx = np.column_stack((X, Y))
        # print("vtx:" , vtx)
        spPoly = Polygon(polygon)
        # print("spPoly", spPoly)
        for x in range(int(min_x), int(max_x) +1):
            for y in range(int(min_y), int(max_y) +1):
                if spPoly.intersects(Point(x,y)):
                    points.append((x,y,layer))
        return points
    def set_obstacle(self, polygons):
        """设置不规则形状的障碍到网格地图."""
        for polygon in polygons:
            X = [ item[0] for item in polygon]
            Y = [ item[1] for item in polygon]
            Z = [ item[2] for item in polygon]
            vtx = np.column_stack((X, Y))
            # print("vtx:" , vtx)
            path = Path(vtx)
            
            # 获取多边形的包络矩形
            min_x, min_y  =  min(X), min(Y)
            max_x, max_y  =  max(X), max(Y)
            layer         =  min(Z)
            for x in range(int(min_x), int(max_x) +1):
                for y in range(int(min_y), int(max_y) +1 ):
                    if path.contains_point((x, y)):
                        self.grid[x, y, layer]   = True  # 在对应层设置障碍
                        self.smrMap[x, y, layer] = 0
    def getSmrValue(self, pt):
        return self.smrMap[pt]
    def is_occupied(self, pt):
        """判断网格是否被占用."""
        # print("pt: ", pt)
        # print("gridStatus: " , self.grid[pt[0], pt[1], pt[2]])
        if (0 <= pt[0] < self.w)  and  (0 <= pt[1] < self.h)  and (0 <= pt[2] < self.l):
           return self.grid[pt[0], pt[1], pt[2]]
        else:
            return False

    def get_neighbors(self, x, y, z, diagonal=False):
        """获取网格的 4 邻域或 8 邻域的顶点."""
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        if diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        neighbors = []
        for dx, dy in directions:
            nx, ny, nz = x + dx, y + dy, z
            if 0 <= nx < self.w and 0 <= ny < self.h and (self.is_occupied((nx, ny, nz)) == False):
                neighbors.append((nx, ny, nz))

        """ layer change """
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
        """绘制 3D 网格地图的视图."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制占用的格子MatplotlibPolygon
        occupied = np.argwhere(self.grid)
        for x, y, z in occupied:
            ax.bar3d(x, y, z, 1, 1, 1, color=colors[z], alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Grid Map')
        plt.show()
     
    def drawSmrMap(self):
        rotated_about_z = np.transpose(self.smrMap, (1, 0, 2))
        plt.figure() 
        for z in range(self.l):
            plt.subplot(self.l, 1, z + 1)
            plt.imshow(rotated_about_z[:, :, z], cmap='viridis', interpolation='nearest', origin='lower')
            plt.title(f'Layer {z + 1}')
            plt.colorbar(label='smrMap')
            plt.xlabel('X')
            plt.ylabel('Y')
        plt.tight_layout()
        # plt.show()
   

    def generate_random_polygon(self, num_vertices):
        """生成一个随机简单多边形."""
        points = set()
    
        while len(points) < num_vertices:
            x = random.randint(0, self.w - 1)
            y = random.randint(0, self.h - 1)
            points.add((x, y))

        # 将点转换为列表并按极角排序，以确保多边形是简单的
        points = list(points)
        centroid = np.mean(points, axis=0)
        points.sort(key=lambda p: np.arctan2(p[1] - centroid[1], p[0] - centroid[0]))
        # 创建多边形
        polygon = Polygon(points)
    
        # 如果多边形无效，重新生成
        if not polygon.is_valid:
            return self.generate_random_polygon(self, num_vertices )
        return polygon

    def generate_random_polygons(self, n, maxVertices=3):
        """在指定范围内生成 n 个随机多边形."""
        polygons = []
        for _ in range(n):
            num_vertices = random.randint(3,maxVertices)  # 随机生成多边形的顶点数，范围可以调整
            polygon = self.generate_random_polygon(num_vertices)
            polygons.append(list(polygon.exterior.coords))
        return polygons

    def generate_random_obstacles(self, num_obstacles=1):
        """在每层随机生成不规则形状的障碍."""
        polygons = []
        for z in range(l):
            polys = self.generate_random_polygons(num_obstacles)
            for poly in polys:
                polygons.append(np.array([(int(x), int(y), int(z)) for (x, y) in poly]))
        return polygons
            
    def setNetTemplate(self, points, lyr, width):
        polygons = self.getPolyShape(points, lyr, width)
        return polygons
  
    def calSmrWithBFS(self, polygons):
        print("size:", (self.w, self.h, self.l))
        # init template as maximum
        startPoints = []
        for polygon in polygons:
            tPoints = self._rasterization(polygon)
            for (x,y,z) in tPoints:
                self.smrMap[x,y,z] = self.s0
            startPoints.extend(tPoints)
        startPoints = set(startPoints)
      
        #visited sets
        visited = set()  # 记录访问过的节点
        directions = [(-1, 0, 0), (1, 0,0), (0, -1,0), (0, 1,0), (0, 0, -1), (0, 0,1)]

        #initialized initial queue
        queue = deque()  # 使用队列存储待访问的节点
        for pt in startPoints:
            queue.append(pt)
        while queue:
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
                if ( 0 <= neighbor[0] < self.h and 0 <= neighbor[1] < self.w and 
                     neighbor not in visited  and  self.grid[neighbor[0],neighbor[1],neighbor[2]] == False):
                    sExist = self.smrMap[neighbor]
                    if neighbor not in startPoints:   
                        queue.append(neighbor)
                        smrVal = self.smrMap[current] -  self.delta - neighbor[2]*5
                        if(smrVal < 0):
                            smrVal = 0
                    else:
                        smrVal =  self.s0
                    
                    # if(smrVal == 0):
                    #     break
                    visited.add(neighbor)
                    self.smrMap[neighbor] = smrVal #max(sExist, smrVal)
 
    def drawShapelyPolygon(self,polygons, w, h):
        fig, ax = plt.subplots(figsize=(w, h)) 
        ax.set_xlim(0, w + 0.1)
        ax.set_ylim(0, h + 0.1)
        """Draw polygon:[(x,y,z), (x1, y1, z1)]"""
        ax.set_aspect('equal')
        ax.set_title('Polygon Visualization')
        for polygon in polygons:
            pts = []
            for (x,y, z) in polygon:
                pts.append([x,y])
            
            p = MatplotlibPolygon(xy=pts, closed=True, fill=True, facecolor=colors[z], edgecolor=colors[z], linewidth=2)
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
    
    #center line [(1,1), (10, 1), (10, 10)]
    def getPolyShape(self, centerLine, lyr, width):
        print("centerLine: ", len(centerLine))
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
 
 
if __name__ == '__main__':    
    # 使用示例
    w, h, l,r = 50, 50, 3,1
    grid = Grid3D(w, h, l, r)
    obses=[]
    # obses = grid.generate_random_obstacles(1)

    """Template generation"""
    path   = grid.setNetTemplate([(1,1),(11,1),(11,10), (20,10)], 0, 2)  
    path1  = grid.setNetTemplate([(30,10),(30,45),(40,45), (40,10),(45, 10), (45, 45)], 0, 2) 

    """Initialize the similarity map with"""
    grid.calSmrWithBFS(path)
    grid.calSmrWithBFS(path1)

    obses.extend(path)
    obses.extend(path1)
    grid.set_obstacle(obses)

    # draw shapely polygon
    grid.drawShapelyPolygon(obses, w, h)
    # grid.draw_grid()
    grid.drawSmrMap()
 
 