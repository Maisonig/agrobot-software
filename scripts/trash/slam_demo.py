import math
import time

import cv2
import numpy as np


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.w = 127
        self.euclidian = 0
        self.neighbors = []


def get_euclidian(n1: Node, n2: Node):
    return math.sqrt((n2.x - n1.x) ** 2 + (n2.y - n1.y) ** 2)


def get_manhattan(n1: Node, n2: Node):
    return abs(n2.x - n1.x) + abs(n2.y - n1.y)


def get_neighbors(node: Node, goal_node: Node):
    std_neighbors = [[0, -1],
                     [1, -1],
                     [1, 0],
                     [1, 1],
                     [0, 1],
                     [-1, 1],
                     [-1, 0],
                     [-1, -1]]
    for neighbor in std_neighbors:
        neighbor_x, neighbor_y = node.x + neighbor[0], node.y + neighbor[1]
        neighbor_node = grid[neighbor_y, neighbor_x]
        neighbor_node.euclidian = get_manhattan(neighbor_node, goal_node)
        node.neighbors.append(grid[neighbor_y, neighbor_x])

    return node.neighbors


def get_closest_neighbor(neighbors: list[Node]):
    closest = neighbors[0]
    for neighbor in neighbors[1:]:
        if neighbor.euclidian < closest.euclidian:
            closest = neighbor
    closest.w = 255
    return closest


def check_last_obstacles():
    global grid
    cur_node = grid[pos[1], pos[0]]
    stop_node = grid[ray[1], ray[0]]
    while stop_node != cur_node:
        neighbors = get_neighbors(cur_node, stop_node)
        cur_node = get_closest_neighbor(neighbors)
    stop_node.w = 0


grid = np.array([[Node(i, j) for i in range(60)] for j in range(60)])
pos = [10, 10]
ray = [55, 44]
check_last_obstacles()

win = np.zeros(shape=(grid.shape[0], grid.shape[1]), dtype=np.uint8)
for i in range(grid.shape[0]):
    for j in range(grid.shape[1]):
        win[j, i] = grid[j, i].w
win = cv2.resize(win, (500, 500))
cv2.imshow("Image", win)
cv2.waitKey(0)
