import time
from collections.abc import Set

import cv2
import numpy as np
import scipy.interpolate as interp

OBSTACLE_VALUE = 255


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def get_euclidian(dx, dy):
    return np.sqrt(dx ** 2 + dy ** 2)


class Grid:

    ONE_CELL_NEIGHBOURS = [(1, 1, 1.4), (0, 1, 1.), (-1, 1, 1.4), (-1, 0, 1), (-1, -1, 1.4), (0, -1, 1.), (1, -1, 1.4), (1, 0, 1.)]
    TWO_CELL_NEIGBOURS = ONE_CELL_NEIGHBOURS + [(1, 2, 1.7), (-1, 2, 1.7), (-2, 1, 1.7), (-2, -1, 1.7), (-1, -2, 1.7), (1, -2, 1.7), (2, -1, 1.7), (2, 1, 1.7)]
    FIVE_CELL_NEIGHBOURS = ONE_CELL_NEIGHBOURS + [(5, 5, 3.1), (-5, 5, 3.1), (-5, -5, 3.1), (5, -5, 3.1), (5, 0, 2.1), (0, 5, 2.1), (-5, 0, 2.1), (0, -5, 2.1)]

    def __init__(self, arr: np.ndarray, neighbours_type=ONE_CELL_NEIGHBOURS):
        self.grid = arr
        self.neighbours = neighbours_type
        self.heuristic = np.zeros_like(arr, dtype=np.float32)
        self.pass_weight = np.zeros_like(arr, dtype=np.float32)
        self.weight = np.zeros_like(arr, dtype=np.float32)

        self.visited = np.zeros_like(arr, dtype=np.bool_)
        self.closed = np.zeros_like(arr, dtype=np.bool_)

        self.parent = np.zeros_like(arr, dtype=tuple)

    def get_neighbours(self, node):
        neighbours = []
        for x, y, p_w in self.neighbours:
            neighbour = [node[0] + x, node[1] + y]
            self.pass_weight[neighbour[1], neighbour[0]] = p_w
            neighbours.append(neighbour)
        return neighbours


class AstarFinder:
    NON_WALKABLE_WEIGHT = 255

    def __init__(self, robot_radius: float, timeout: float = 1., goal_radius: float = 1., smooth: bool = False):
        self.goalRad = goal_radius
        self.robotRad = robot_radius
        self.timeout = timeout
        self.smooth = smooth

        self.circleRad = 1 + int(np.sqrt(self.robotRad**2 + self.robotRad**2))
        self.circleThick = int(self.circleRad - self.robotRad)

    def check_collision(self, grid, node):
        col = np.copy(grid.grid[int(node[1] - self.robotRad):int(node[1] + self.robotRad), int(node[0] - self.robotRad):int(node[0] + self.robotRad)])
        cv2.circle(col, [int(self.robotRad), int(self.robotRad)], self.circleRad, [127], self.circleThick * 2)
        try:
            if np.max(col) == 255:
                return False
            else:
                return True
        except:
            return False

    def get_path(self, grid, start_pos, goal_pos):

        x0 = start_pos[0]
        y0 = start_pos[1]
        x1 = goal_pos[0]
        y1 = goal_pos[1]

        grid.heuristic[y0, x0] = get_euclidian(abs(x0 - x1), abs(y0 - y1))
        grid.weight[y0, x0] = grid.heuristic[y0, x0] + grid.grid[y0, x0]

        open_list = [start_pos]
        open_list_weights = [grid.weight[y0, x0]]
        closed_list = []

        start_time = time.time()

        while True:
            # Поиск индекса минимального веса, обозначение ячейки как текущей и удаление ячейки и веса из открытого
            # списка
            min_index = open_list_weights.index(min(open_list_weights))
            current = open_list[min_index]
            open_list.pop(min_index)
            open_list_weights.pop(min_index)
            grid.visited[current[1], current[0]] = True
            #
            closed_list.append(current)
            grid.closed[current[1], current[0]] = True

            # Если достигли финишной ноды, прерываем цикл и возвращаем путь
            if x1 - self.goalRad < current[0] < x1 + self.goalRad:
                if y1 - self.goalRad < current[1] < y1 + self.goalRad:
                    return self.reconstruct_path(grid, closed_list)

            # Если за время таймаута путь не обнаружен, прерываем цикл и ничего не возвращаем
            if time.time() > start_time + self.timeout:
                return None

            neighbours = grid.get_neighbours(current)
            for neighbour in neighbours:
                x = neighbour[0]
                y = neighbour[1]
                grid_weight = grid.grid[y, x]
                if grid_weight == self.NON_WALKABLE_WEIGHT or grid.closed[y, x] or not self.check_collision(grid, neighbour):
                    pass
                else:
                    heuristic = get_euclidian(abs(x - x1), abs(y - y1))
                    total = heuristic + grid.pass_weight[y, x] + grid_weight
                    if total < grid.weight[current[1], current[0]] or not grid.visited[y, x]:
                        grid.heuristic[y, x] = heuristic
                        grid.weight[y, x] = total
                        grid.parent[y, x] = (current[0], current[1])
                        if not grid.visited[y, x]:
                            grid.visited[y, x] = True
                            open_list.append(neighbour)
                            open_list_weights.append(total)

    def reconstruct_path(self, grid, nodes):
        p = []
        nodes = list(reversed(nodes))
        start = nodes[0]
        while start != nodes[-1]:
            p.append(grid.parent[start[1], start[0]])
            start = (grid.parent[start[1], start[0]])
        p = list(reversed(p))
        p.append(nodes[0])
        if self.smooth:
            polyline = np.array(p)
            smoothed = self.smooth_path(polyline, int(len(p) / 5))
            if smoothed is not None:
                p = list(smoothed)
        return p

    def smooth_path(self, polyline, num_points):
        tck, u = interp.splprep([polyline[:, 0], polyline[:, 1]], s=1)
        u = np.linspace(0.0, 1.0, num_points)
        p = np.column_stack(interp.splev(u, tck))
        if np.isnan(polyline.min()):
            return None
        if p.min() > 0:
            return p
        else:
            return None



def mouse_click(event, x, y,
                flags, param):
    global mouseX, mouseY, a, drawA
    if event == cv2.EVENT_MOUSEMOVE:
        mouseX, mouseY = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        a = cv2.circle(a, [x, y], 30, [255], -1)
        drawA = cv2.circle(drawA, [x, y], 30, [255], -1)


cv2.namedWindow("Grid")
cv2.setMouseCallback('Grid', mouse_click)
a = np.zeros([1000, 1000], np.uint8)
a = cv2.circle(a, [20, 80], 15, [255], -1)
a = cv2.circle(a, [40, 85], 15, [255], -1)
drawA = np.copy(a)
gr = Grid(a, Grid.FIVE_CELL_NEIGHBOURS)
finder = AstarFinder(10, smooth=True)
mouseX, mouseY = 50, 150


while True:
    gr.__init__(a)
    start_time = time.time()
    path = finder.get_path(gr, (40, 40), (mouseX, mouseY))

    if path is not None:
        for point in path:
            cv2.circle(drawA, [int(point[0]), int(point[1])], 1, [200], -1)
    cv2.imshow("Grid", drawA)
    cv2.waitKey(1)
    drawA = np.copy(a)
    # print(1 / (time.time() - start_time))
