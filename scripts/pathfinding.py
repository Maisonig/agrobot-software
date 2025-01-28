import time

import cv2
import numpy as np


OBSTACLE_VALUE = 255


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def get_euclidian(dx, dy):
    return np.sqrt(dx**2 + dy**2)


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.parent = None

        self.visited = False
        self.closed = False

        self.heuristic = 0
        self.pass_weight = 0
        self.weight = 0

    def set_total_weight(self, grid_weight):
        self.weight = grid_weight + self.heuristic + self.pass_weight


class Grid:
    def __init__(self, arr: np.ndarray):
        self.grid = arr
        self.nodes = self.create_nodes()

    def refresh_grid(self, grid):
        self.grid = grid
        self.nodes = self.create_nodes()

    def create_nodes(self):
        nodes = []
        for i in range(self.grid.shape[0]):
            row = []
            for j in range(self.grid.shape[1]):
                n = Node(j, i)
                row.append(n)
            nodes.append(row)
        ns = np.array(nodes)
        return ns

    def get_neighbours(self, node):
        neighbours = []
        for x, y, p_w in [(1, 1, 1.4), (0, 1, 1.), (-1, 1, 1.4), (-1, 0, 1), (-1, -1, 1.4), (0, -1, 1.), (1, -1, 1.4), (1, 0, 1.)]:
            neighbour = self.nodes[node.y + y, node.x + x]
            neighbour.pass_weight = p_w
            neighbours.append(neighbour)
        return neighbours


class AstarFinder:

    NON_WALKABLE_WEIGHT = 255

    def __init__(self, robot_radius: float, timeout: float = 1., goal_radius: float = 1.):
        self.goalRad = goal_radius
        self.robotRad = robot_radius
        self.timeout = timeout

    def check_collision(self, grid, node):
        col = grid.grid[node.y - self.robotRad:node.y + self.robotRad, node.x - self.robotRad:node.x + self.robotRad]
        try:
            if col.max() == 255:
                return False
            else:
                return True
        except:
            return False

    def get_lowest_weight(self, nodes):
        lowest = nodes[0]
        for node in nodes:
            if node.weight < lowest.weight:
                lowest = node
        return lowest

    def get_path(self, grid, start_pos, goal_pos):

        x0 = start_pos[0]
        y0 = start_pos[1]
        x1 = goal_pos[0]
        y1 = goal_pos[1]

        start = grid.nodes[y0, x0]
        goal = grid.nodes[y1, x1]

        start.heuristic_weight = get_euclidian(abs(x0 - x1), abs(y0 - y1))
        start.set_total_weight(grid.grid[y0, x0])

        open_list = [start]
        closed_list = []

        start_time = time.time()

        while True:
            current = self.get_lowest_weight(open_list)
            open_list.remove(current)
            current.visited = True
            closed_list.append(current)
            current.closed = True

            # Если достигли финишной ноды, прерываем цикл и возвращаем путь
            if x1 - self.goalRad < current.x < x1 + self.goalRad:
                if y1 - self.goalRad < current.y < y1 + self.goalRad:
                    return self.reconstruct_path(closed_list)

            # Если за время таймаута путь не обнаружен, прерываем цикл и ничего не возвращаем
            if time.time() > start_time + self.timeout:
                return None

            neighbours = grid.get_neighbours(current)
            for neighbour in neighbours:
                grid_weight = grid.grid[neighbour.y, neighbour.x]
                if grid_weight == self.NON_WALKABLE_WEIGHT or neighbour.closed or not self.check_collision(grid, neighbour):
                    pass
                else:
                    heuristic = get_euclidian(abs(neighbour.x - x1), abs(neighbour.y - y1))
                    total = heuristic + neighbour.pass_weight + grid_weight
                    if total < current.weight or not neighbour.visited:
                        neighbour.heuristic = heuristic
                        neighbour.set_total_weight(grid_weight)
                        neighbour.parent = current
                        if not neighbour.visited:
                            neighbour.visited = True
                            open_list.append(neighbour)

    def reconstruct_path(self, nodes):
        p = []
        nodes = list(reversed(nodes))
        start = nodes[0]
        while start != nodes[-1]:
            p.append(start.parent)
            start = start.parent
        p = list(reversed(p))
        p.append(nodes[0])
        return p


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
a = np.zeros([600, 600], np.uint8)
a = cv2.circle(a, [20, 80], 15, [255], -1)
a = cv2.circle(a, [40, 85], 15, [255], -1)
drawA = np.copy(a)
gr = Grid(a)
finder = AstarFinder(20)
mouseX, mouseY = 50, 150


while True:
    start_time = time.time()

    gr.refresh_grid(a)
    path = finder.get_path(gr, (40, 40), (mouseX, mouseY))
    
    if path is not None:
        for point in path:
            cv2.circle(drawA, [point.x, point.y], 1, [127], -1)
        cv2.imshow("Grid", drawA)
        cv2.waitKey(1)
    drawA = np.copy(a)
    cv2.imshow("Grid", drawA)
    cv2.waitKey(1)

    print(1 / (time.time() - start_time))


