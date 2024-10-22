import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
from matplotlib.path import Path
from shapely.geometry import Polygon
# 定义颜色
colors = ["red", "blue", "magenta", "green", "deepskyblue", "gold", "lime"]
 
    
class Grid3D:
    def __init__(self, w, h, l, r):
        self.w = w  # 长度
        self.h = h  # 高度
        self.l = l  # 层数
        self.r = r  # resolution

        self.grid   = np.zeros((w, h, l), dtype=bool)  # 用于存储占用状态
        self.smrMap = np.zeros((w, h, l), dtype=float)  # save similarity map

    def set_obstacle(self, polygons):
        """设置不规则形状的障碍到网格地图."""
        for polygon in polygons:
            path = Path(polygon[:, :2])  # 只使用 x 和 y 坐标
            # 获取多边形的包络矩形
            min_x, min_y = polygon[:, 0].min(), polygon[:, 1].min()
            max_x, max_y = polygon[:, 0].max(), polygon[:, 1].max()
            layer        = polygon[:, 2].min()
            for x in range(int(min_x), int(max_x) + 1):
                for y in range(int(min_y), int(max_y) + 1):
                    if path.contains_point((x, y)):
                        self.grid[x, y, layer] = True  # 在对应层设置障碍

    def is_occupied(self, x, y, z):
        """判断网格是否被占用."""
        return 0 <= x < self.w and 0 <= y < self.h and 0 <= z < self.l and self.grid[x, y, z]

    def get_neighbors(self, x, y, z, diagonal=False):
        """获取网格的 4 邻域或 8 邻域的顶点."""
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        if diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        neighbors = []
        for dx, dy in directions:
            nx, ny, nz = x + dx, y + dy, z
            if 0 <= nx < self.w and 0 <= ny < self.h:
                neighbors.append((nx, ny, nz))

        """ layer change """
        layers = [-1, 1]
        for dz in layers:
            nz = z + dz
            neighbors.append((x,y, nz))

        return neighbors

    def draw_grid(self):
        """绘制 3D 网格地图的视图."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制占用的格子
        occupied = np.argwhere(self.grid)
        for x, y, z in occupied:
            ax.bar3d(x, y, z, 1, 1, 1, color=colors[z], alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Grid Map')
        plt.show()

    def draw_2d_map(self):
        """绘制 2D 地图，不同层以不同颜色表示."""
        plt.figure(figsize=(10, 8))
        for z in range(self.l):
            plt.subplot(self.l, 1, z + 1)
            plt.imshow(self.grid[:, :, z], cmap='Grays', alpha=0.5)
            plt.title(f'Layer {z + 1}')
            plt.colorbar(label='Occupied')
            plt.xlabel('X')
            plt.ylabel('Y')
        
        plt.tight_layout()
        plt.show()

 
def generate_random_polygon(num_vertices, grid_width, grid_height):
    """生成一个随机简单多边形."""
    points = set()
    
    while len(points) < num_vertices:
        x = random.randint(0, grid_width - 1)
        y = random.randint(0, grid_height - 1)
        points.add((x, y))

    # 将点转换为列表并按极角排序，以确保多边形是简单的
    points = list(points)
    centroid = np.mean(points, axis=0)
    points.sort(key=lambda p: np.arctan2(p[1] - centroid[1], p[0] - centroid[0]))

    # 创建多边形
    polygon = Polygon(points)
    
    # 如果多边形无效，重新生成
    if not polygon.is_valid:
        return generate_random_polygon(num_vertices, grid_width, grid_height)
    
    return polygon

def generate_random_polygons(n, w, h):
    """在指定范围内生成 n 个随机多边形."""
    polygons = []
    
    for _ in range(n):
        num_vertices = random.randint(3, 8)  # 随机生成多边形的顶点数，范围可以调整
        polygon = generate_random_polygon(num_vertices, w, h)
        polygons = list(polygon.exterior.coords)
    return polygons

def generate_random_obstacles(grid, num_obstacles=3):
    """在每层随机生成不规则形状的障碍."""
    for z in range(grid.l):
        polygons = []
        for _ in range(num_obstacles):
            points = generate_random_polygons(num_obstacles, grid.w, grid.h)
            polygons.append(np.array([(int(x), int(y), int(z)) for (x, y) in points]))
        grid.set_obstacle(polygons)
# 使用示例
w, h, l,r = 50, 50, 3,1
grid = Grid3D(w, h, l, r)
generate_random_obstacles(grid)
# grid.draw_grid()
grid.draw_2d_map()
