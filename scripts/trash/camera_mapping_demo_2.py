import os
import cv2
import math
import time
import numpy as np


def mouse_click(event, x, y,
                flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        print(x, y)


def get_hsv(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    return img


def get_color_segmented(img):
    lower_range = np.array((12, 25, 25))  # lower range of green color in HSV
    upper_range = np.array((86, 255, 255))  # upper range of green color in HSV
    msk = cv2.inRange(img, lower_range, upper_range)
    img = cv2.bitwise_and(img, img, mask=msk)
    return img, msk


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
    h = 10
    slices = int(img.shape[0] / h) + 1
    slices_coord = [[h * i, h * i + h] for i in range(slices)]
    node_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    g = []
    j = slices
    for j in range(slices):
        row = img[slices_coord[j][0]:slices_coord[j][1], :]
        contours, hierarchy = cv2.findContours(row, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = list(contours)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            x, y, width, height = cv2.boundingRect(contours[i])
            x, y = int(x + width / 2), int((h * j + y) + height / 2)
            if area > 100:
                # if g:
                #     if abs(x - g[-1].x) > 100:
                #         cv2.circle(node_img, [x, y], 5, (255, 255, 0), -1)
                #         g.append(Node(x, y))
                #     else:
                #         x = int((x + g[-1].x) / 2)
                #         g.pop(-1)
                #         cv2.circle(node_img, [x, y], 5, (255, 0, 0), -1)
                #         g.append(Node(x, y))
                # else:
                cv2.circle(node_img, [x, y], 5, (255, 255, 0), -1)
                g.append(Node(x, y))
        j -= 1
    return node_img, g


def get_row_path(img, g):
    ps = []
    start_height = 20
    starts = []
    for node in g:
        if node.y > img.shape[0] - start_height:
            starts.append(node)
    start = starts[0]
    for node in starts[1:]:
        if start.x + 200 > node.x > start.x - 200:
            starts.remove(node)
    if starts is None:
        return img, None
    if len(starts) == 1:
        return img, None
    if len(starts) == 2:
        if starts[1].x < starts[0].x:
            starts = list(reversed(starts))
    if len(starts) > 2:
        left = Node(0, 0)
        right = Node(img.shape[1], 0)
        for node in starts:
            if left.x < node.x < img.shape[1] / 2:
                left = node
        for node in starts:
            if left.x + 250 < node.x:
                if right.x > node.x > img.shape[1] / 2:
                    if node.x < right.x:
                        right = node
        starts = [left, right]
        for node in starts:
            node.marked = True
        if right.x == img.shape[1]:
            return img, None

    min_dst = img.shape[1]
    for start in starts:
        p = []
        while start.y > img.shape[0] / 2:
            cur = start
            for node in g:
                if node.marked is False:
                    if node.y < start.y:
                        if start.x - 100 < node.x < start.x + 100:
                            dst = math.dist([node.x, node. y], [start.x, start.y])
                            if dst < min_dst:
                                min_dst = dst
                                cur = node
            if cur == start:
                break
            cv2.line(img, [start.x, start.y], [cur.x, cur.y], (127, 127, 0), 2)
            start = cur
            start.marked = True
            p.append(start)
            cv2.circle(img, [start.x, start.y], 5, (255, 0, 0), 2)
            min_dst = img.shape[1]
        ps.append(p)

    return img, ps


def get_paths_straights(img, ps):
    ss = []
    for p in ps:
        if len(p) > 4:
            x_1 = p[1].x
            y_1 = p[1].y
            x_2 = p[-1].x
            y_2 = p[-1].y
            node1 = Node(x_1, y_1)
            node2 = Node(x_2, y_2)
            s = Straight(node1.x, node1.y, node2.x, node2.y)
            y_1 = img.shape[0]
            x_1 = int(s.calc_x(y_1))
            y_2 = int(img.shape[0] / 2)
            x_2 = int(s.calc_x(y_2))
            cv2.line(img, (x_1, y_1), (x_2, y_2), (139, 0, 0), 9)
            cv2.line(img, (x_1, y_1), (x_2, y_2), (255, 0, 0), 5)
            ss.append(s)
    return img, ss


def get_true_straights(img, ss, true_ss):
    acc = 0
    for s, t_s in zip(ss, true_ss):
        y_1 = img.shape[0]
        x_1 = int(t_s.calc_x(y_1))
        y_2 = int(img.shape[0] / 2)
        x_2 = int(t_s.calc_x(y_2))
        cv2.line(img, (x_1, y_1), (x_2, y_2), (0, 0, 139), 9)
        cv2.line(img, (x_1, y_1), (x_2, y_2), (0, 0, 255), 5)
        acc += (s.get_accuracy(t_s, img.shape[1]))
    # print(acc / 2)
    return img


def get_canny(img):
    img = cv2.Canny(img, 50, 200, None, 3)
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


class Straight(object):
    def __init__(self, x_1, y_1, x_2, y_2):
        self.x1 = x_1
        self.y1 = y_1
        self.x2 = x_2
        self.y2 = y_2

        self.k = 0.
        self.b = 0.
        self.get_canonical_equation_coefficients()

        self.accuracy = 0.

    def get_canonical_equation_coefficients(self):
        if self.x1 == self.x2:
            self.x1 = self.x1 - 0.001
        if self.y1 == self.y2:
            self.y1 = self.y1 - 0.001
        self.k = (self.y1 - self.y2) / (self.x1 - self.x2)
        self.b = self.y2 - self.k * self.x2

    def calc_y(self, x):
        return self.k * x + self.b

    def calc_x(self, y):
        return (y - self.b) / self.k

    def get_accuracy(self, true, normalizing=None):
        y1 = 500
        y2 = 1000
        x1 = self.calc_x(y1)
        x2 = self.calc_x(y2)
        x1_true = true.calc_x(y1)
        x2_true = true.calc_x(y2)
        if normalizing:
            x1_acc = 1 - abs(abs(x1 - x1_true) / normalizing)
            x2_acc = 1 - abs(abs(x2 - x2_true) / normalizing)
        else:
            x1_acc = 1 - abs(abs(x1 - x1_true) / x1_true)
            x2_acc = 1 - abs(abs(x2 - x2_true) / x2_true)
        self.accuracy = (x1_acc + x2_acc) / 2
        return self.accuracy


class Image(object):
    def __init__(self, img: np.ndarray, true_straights):
        self.width = img.shape[0]
        self.height = img.shape[1]
        self.data = img

        self.transformed = None

        self.true_straights = true_straights
        self.straights = []


annotations = [
    [Straight(295, 716, 490, 298), Straight(939, 716, 790, 319)],
    [Straight(322, 859, 460, 390), Straight(820, 860, 700, 400)],
    [Straight(522, 663, 525, 320), Straight(891, 665, 718, 351)],
    [Straight(216, 617, 371, 323), Straight(631, 645, 587, 324)],
    [Straight(423, 646, 480, 261), Straight(763, 641, 643, 344)],
    [Straight(290, 569, 343, 292), Straight(653, 563, 530, 299)],
    [Straight(295, 716, 490, 298), Straight(939, 716, 790, 319)],
    [Straight(522, 663, 525, 320), Straight(891, 665, 718, 351)],
    [Straight(216, 617, 371, 323), Straight(631, 645, 587, 324)],
    [Straight(423, 646, 480, 261), Straight(763, 641, 643, 344)],
    [Straight(290, 569, 343, 292), Straight(653, 563, 530, 299)],
    [Straight(295, 716, 490, 298), Straight(939, 716, 790, 319)],
    [Straight(522, 663, 525, 320), Straight(891, 665, 718, 351)],
    [Straight(216, 617, 371, 323), Straight(631, 645, 587, 324)],
    [Straight(423, 646, 480, 261), Straight(763, 641, 643, 344)],
    [Straight(290, 569, 343, 292), Straight(653, 563, 530, 299)],
    [Straight(295, 716, 490, 298), Straight(939, 716, 790, 319)],
    [Straight(522, 663, 525, 320), Straight(891, 665, 718, 351)],

]
images = []
print(os.listdir("./images"))
paths = [p for p in os.listdir("./images") if p.endswith(".png")]
paths.sort()
nums = list()
for path, st in zip(paths, annotations):
    data = cv2.imread("./images/" + path)
    images.append(Image(data, st))


image_num = 0
process_type = 0
process_num = 0


finish = False
while not finish:
    start_time = time.time()
    imageObj = images[image_num]
    image = np.copy(imageObj.data)
    canvas = np.copy(imageObj.data)

    if process_num == 0:
        pass
    if process_num == 1:
        image = get_hsv(image)
    if process_num == 2:
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
    if process_num == 3:
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = mask
    if process_num == 4:
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = get_noiseless(image)
    if process_num == 5:
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = mask
        image = get_noiseless(image)
    if process_num == 6:
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = get_noiseless(image)
        image = get_max_pooled(image)
    if process_num == 7:
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = mask
        image = get_noiseless(image)
        image = get_max_pooled(image)
        image, graph = get_rows_nodes(image)
        image, paths = get_row_path(image, graph)
    if process_num == 8:
        source = np.copy(image)
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = mask
        image = get_noiseless(image)
        image = get_max_pooled(image)
        image, graph = get_rows_nodes(image)
        image, paths = get_row_path(image, graph)
        image, imageObj.straights = get_paths_straights(source, paths)
    if process_num == 9:
        source = np.copy(image)
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        image = mask
        image = get_noiseless(image)
        image = get_max_pooled(image)
        image, graph = get_rows_nodes(image)
        image, paths = get_row_path(image, graph)
        source, imageObj.straights = get_paths_straights(source, paths)
        image = get_true_straights(source, imageObj.straights, imageObj.true_straights)
        print(1.0 / (time.time() - start_time))

    cv2.imshow("figure", image)
    cv2.setMouseCallback('figure', mouse_click)
    key = cv2.waitKey(1)
    # 82 - up 119 246
    # 84 - down 115 251
    # 81 - left 97 244
    # 83 - right 100 226
    if key == 119 or key == 246:
        process_num += 1
        if process_num > 9:
            process_num = 9
    elif key == 115 or key == 251:
        process_num -= 1
        if process_num < 0:
            process_num = 0
    elif key == 100 or key == 226:
        image_num += 1
        process_num = 0
        if image_num > len(images) - 1:
            image_num = len(images) - 1
    elif key == 97 or key == 244:
        image_num -= 1
        process_num = 0
        if image_num < 0:
            image_num = 0
    elif key == 27:
        finish = True

    if key != -1:
        print(process_num)
        print(key)
