import math
import time

import cv2
import numpy as np


def get_perspective(img):
    real_pts = np.float32([
        [770, 700],
        [840, 700],
        [840, 710],
        [770, 710],

    ])
    img_pts = np.float32([
        [400, 500],
        [855, 500],
        [958, 719],
        [300, 719],

    ])
    h, status = cv2.findHomography(img_pts, real_pts)
    perspective = cv2.warpPerspective(img, h, [1600, 710])
    return perspective


def get_hsv(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    return img


def get_gray(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    return img


def get_color_segmented(img):
    lower_range = np.array((12, 25, 25))  # lower range of green color in HSV
    upper_range = np.array((86, 255, 255))  # upper range of green color in HSV
    mask = cv2.inRange(img, lower_range, upper_range)
    img = cv2.bitwise_and(img, img, mask=mask)
    return img, mask


def get_noiseless(img):
    img = cv2.medianBlur(img, 5)
    kernel = np.ones((5, 5), np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return img


def get_max_pooled(img):
    w, h = img.shape[1], img.shape[0]
    img = cv2.resize(img, (int(w / 7), int(h / 7)))
    img = cv2.resize(img, (w, h))
    img[img > 0] = 255
    return img


def get_rows_nodes(img):
    h = 80
    slices = int(img.shape[0] / 50)
    slices_coord = [[h * i, h * i + h] for i in range(slices)]
    node_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    graph = []
    j = slices
    for j in range(slices):
        row = img[slices_coord[j][0]:slices_coord[j][1], :]
        contours, hierarchy = cv2.findContours(row, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = list(contours)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            x, y, width, height = cv2.boundingRect(contours[i])
            x1, y1 = int(x + width / 2), int((h * j + y) + height / 2)
            y = y + h * j
            if area < 500:
                cv2.rectangle(node_img, [x, y], [x + width, y + height], (0, 0, 0), -1)
            else:
                cv2.circle(node_img, [x1, y1], 5, (255, 0, 0), -1)
                graph.append(Node(x1, y1))
        j -= 1
    return node_img, graph


def get_row_path(img, graph):
    paths = []

    for i in range(3):
        p = []
        start = Node(0, 0)
        for node in graph:
            if node.marked is False:
                if paths:
                    flg = 0
                    for p in paths:
                        if abs(p[0].x - node.x) > 300:
                            flg += 1
                        else:
                            break
                        if flg == len(paths):
                            if node.y > start.y:
                                if img.shape[1] / 2 - 500 < node.x < img.shape[1] / 2 + 500:
                                    start = node
                else:
                    if node.y > start.y:
                        if img.shape[1] / 2 - 500 < node.x < img.shape[1] / 2 + 500:
                            start = node
        start.marked = True
        p.append(start)
        cv2.circle(img, [start.x, start.y], 5, (255, 255, 0), 2)
        min_dst = img.shape[1]
        while start.y > img.shape[0] / 2:
            cur = start
            for node in graph:
                if node.marked is False:
                    if node.y < start.y:
                        if start.x - 100 < node.x < start.x + 100:
                            dst = math.dist([node.x, node. y], [start.x, start.y])
                            if dst < min_dst:
                                min_dst = dst
                                cur = node
            cv2.line(img, [start.x, start.y], [cur.x, cur.y], (127, 127, 0), 2)
            start = cur
            start.marked = True
            p.append(start)
            cv2.circle(img, [start.x, start.y], 5, (255, 0, 255), 2)
            min_dst = img.shape[1]
        if len(p) > 1:
            paths.append(p)

    if len(paths) == 2:
        if paths[0][0].x > paths[1][0].x:
            p = paths[0]
            paths[0] = paths[1]
            paths[1] = p

    return img, paths


def get_straight_equation(x1, y1, x2, y2):
    if x1 == x2:
        return 1, None
    elif y1 == y2:
        return None, 1
    else:
        k = (y1 - y2) / (x1 - x2)
        b = y2 - k * x2
        return k, b


def get_lines_accuracy(kb):
    acc = []
    for kb_, kb_true_ in zip(kb, kb_true):
        k_acc = 1 - abs(abs(kb_[0] - kb_true_[0])/kb_true_[0])
        b_acc = 1 - abs(abs(kb_[1] - kb_true_[1])/kb_true_[1])
        acc.append((k_acc + b_acc) / 2)
    accuracy = sum(acc) / len(acc)
    print(accuracy)


def get_rows_lines(img, ps):
    kb = []
    for p1 in ps:
        cv2.line(img, [p1[0].x, p1[0].y], [p1[-1].x, p1[-1].y], (0, 0, 255), 6)
        kb.append(get_straight_equation(p1[0].x, p1[0].y, p1[-1].x, p1[-1].y))
    get_lines_accuracy(kb)
    return img


def get_rows_straights(img, kb):
    for k, b in kb:
        y = 719
        x = int((y - b) / k)
        p1 = Node(x, y)
        y = 400
        x = int((y - b) / k)
        p2 = Node(x, y)
        cv2.line(img, [p1.x, p1.y], [p2.x, p2.y], (255, 0, 0), 6)
    return img


def get_canny(img):
    img = cv2.Canny(img, 50, 200, None, 3)
    return img


def get_empty_graph(img, graph):
    img = np.zeros_like(img)
    for node in graph:
        cv2.circle(img, [node.x, node.y], 1, (255, 255, 255), -1)
    return img


def get_hough_lines(img):
    lines = cv2.HoughLines(img, 200, np.pi / 180, 150, None, 0, 0)
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            cv2.line(img, pt1, pt2, (255, 255, 255), 3, cv2.LINE_AA)
    return img


class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.marked = False


image_list = {
    'images': [],
    'kb_true': [],
}

image_list['images'].append(cv2.imread("../i0.png"))
koef1 = get_straight_equation(293, 719, 442, 402)
koef2 = get_straight_equation(940, 717, 793, 323)
image_list['kb_true'].append([koef1, koef2])

image_list['images'].append(cv2.imread("../i1.png"))
koef1 = get_straight_equation(681, 719, 650, 363)
koef2 = get_straight_equation(1277, 562, 1045, 363)
image_list['kb_true'].append([koef1, koef2])

image_list['images'].append(cv2.imread("../i2.png"))
koef1 = get_straight_equation(621, 718, 623, 360)
koef2 = get_straight_equation(1062, 703, 840, 360)
image_list['kb_true'].append([koef1, koef2])

image_list['images'].append(cv2.imread("../i3.png"))
koef1 = get_straight_equation(621, 718, 623, 360)
koef2 = get_straight_equation(1062, 703, 840, 360)
image_list['kb_true'].append([koef1, koef2])

image_list['images'].append(cv2.imread("../i4.png"))
koef1 = get_straight_equation(621, 718, 623, 360)
koef2 = get_straight_equation(1062, 703, 840, 360)
image_list['kb_true'].append([koef1, koef2])

image_list['images'].append(cv2.imread("../i5.png"))
koef1 = get_straight_equation(621, 718, 623, 360)
koef2 = get_straight_equation(1062, 703, 840, 360)
image_list['kb_true'].append([koef1, koef2])

# perspective_image = get_perspective(source_image)
# source_image_roi = source_image[150:, :]
# perspective_image_roi = perspective_image[perspective_image.shape[0] - 260:, int(perspective_image.shape[1] / 2 - 150):int(perspective_image.shape[1] / 2 + 150)]

image_num = 0
process_type = 0
process_num = 0


finish = False
while not finish:
    start_time = time.time()
    image = image_list.get('images')[image_num]
    kb_true = image_list.get('kb_true')[image_num]
    image = cv2.resize(image, (1280, 720))
    if process_type == 0:
        if process_num == 0:
            pass
        if process_num == 1:
            image = get_hsv(image)
        if process_num == 2:
            image = get_gray(image)
        if process_num == 3:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
        if process_num == 4:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
        if process_num == 5:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = get_noiseless(image)
        if process_num == 6:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = get_noiseless(mask)
        if process_num == 7:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = get_noiseless(image)
            image = get_max_pooled(image)
        if process_num == 8:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
        if process_num == 9:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
            image, path = get_row_path(image, graph_)
        if process_num == 10:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
            image, paths_ = get_row_path(image, graph_)
            image = get_rows_lines(image, paths_)
        if process_num == 11:
            color_image = np.copy(image)
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
            image, paths_ = get_row_path(image, graph_)
            image = get_rows_lines(color_image, paths_)
        if process_num == 12:
            color_image = np.copy(image)
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
            image, paths_ = get_row_path(image, graph_)
            image = get_rows_lines(color_image, paths_)
            image = get_rows_straights(image, kb_true)
    if process_type == 1:
        if process_num == 0:
            pass
        if process_num == 1:
            image = get_hsv(image)
        if process_num == 2:
            image = get_gray(image)
            image = get_canny(image)
            image = get_hough_lines(image)
        if process_num == 3:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
        if process_num == 4:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
        if process_num == 5:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = get_noiseless(image)
        if process_num == 6:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
            image = get_empty_graph(image, graph_)
        if process_num == 7:
            image = get_hsv(image)
            image, mask = get_color_segmented(image)
            image = mask
            image = get_noiseless(image)
            image = get_max_pooled(image)
            image, graph_ = get_rows_nodes(image)
            image = get_empty_graph(image, graph_)
            image = get_canny(image)
            image = get_hough_lines(image)

    # print(1.0 / (time.time() - start_time))

    cv2.imshow("figure", image)
    key = cv2.waitKey(1)
    # 82 - up
    # 84 - down
    # 81 - left
    # 83 - right
    if key != -1:
        print(process_num)
    if key == 82:
        process_num += 1
        if process_num > 12:
            process_num = 12
    elif key == 84:
        process_num -= 1
        if process_num < 0:
            process_num = 0
    elif key == 83:
        image_num += 1
        process_num = 0
        if image_num > 4:
            image_num = 4
    elif key == 81:
        image_num -= 1
        process_num = 0
        if image_num < 0:
            image_num = 0
    elif key == 27:
        finish = True
