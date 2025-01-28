import time

import cv2
import random
import numpy as np

from numpy.linalg import LinAlgError


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.w = 127
        self.euclidian = 0
        self.neighbors = []


def solve_equation_of_a_straight(n1: Node, n2: Node):
    variables = np.array([[n1.x, 1.], [n2.x, 1.]])
    free = np.array([n1.y, n2.y])
    try:
        return np.linalg.solve(variables, free)
    except LinAlgError:
        if n1.x == n2.x:
            return [0, n1.x]
        else:
            return [0, n1.y]


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


def check_last_obstacles():
    global grid
    cur_node = grid[pos[1], pos[0]]
    stop_node = grid[ray[1], ray[0]]
    k, b = solve_equation_of_a_straight(cur_node, stop_node)
    try:
        x_koef = (stop_node.x - cur_node.x) / abs(stop_node.x - cur_node.x)
        y_koef = (stop_node.y - cur_node.y) / abs(stop_node.y - cur_node.y)
    except:
        if cur_node.y > stop_node.y:
            x_koef = -1
        else:
            x_koef = 1
        if cur_node.x > stop_node.x:
            y_koef = -1
        else:
            y_koef = 1
    dx = abs(cur_node.x - stop_node.x)
    dy = abs(cur_node.y - stop_node.y)
    if k < 0.001 and k > -0.001:
        k = 0
    while not ((stop_node.x - 1 <= cur_node.x and stop_node.x + 1 >= cur_node.x) and (stop_node.y - 1 <= cur_node.y and stop_node.y + 1 >= cur_node.y)):
        if k != 0 and dx > dy:
            x = int(cur_node.x + 1 * x_koef)
            y = int(k * x + b)
        elif k != 0 and dy >= dx:
            y = int(cur_node.y + 1 * y_koef)
            x = int((y - b) / k)
        elif cur_node.x == stop_node.x:
            x = int(cur_node.x)
            y = int(cur_node.y + 1 * x_koef)
        else:
            x = int(cur_node.x + 1 * y_koef)
            y = cur_node.y
        cur_node.w = 255
        cur_node = grid[y, x]

    stop_node.w = 0


grid = np.array([[Node(i, j) for i in range(60)] for j in range(60)])
pos = [29, 29]
ray = [29, 5]

n = 0
sum_fps = 0
while True:
    start_time = time.time()
    ray = [random.randint(0, 59), random.randint(0, 59)]
    if ray == pos:
        ray[0] = ray[0] + 1

    check_last_obstacles()

    win = np.zeros(shape=(grid.shape[0], grid.shape[1]), dtype=np.uint8)
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            win[j, i] = grid[j, i].w
    win = cv2.resize(win, (500, 500))
    cv2.imshow("Image", win)
    cv2.waitKey(1)
    print("FPS: ", 1.0 / (time.time() - start_time))
    sum_fps += 1.0 / (time.time() - start_time)
    n += 1
    if n == 10000:
        print(sum_fps / n)
        break
