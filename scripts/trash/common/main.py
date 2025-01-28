import os
import cv2
import numpy as np
from tensorflow import keras


FIGURES_PATH = "figures/"
IMAGES_PATH = "images/"
COLOR_IMAGE_NAME = "img_00333.png"
DEPTH_IMAGE_NAME = "depth_00333.png"
MASK_IMAGE_NAME = "mask_00333.png"
MODEL_PATH = "/"
MODEL_NAME = "model.h5"

CAMERA_VERTICAL_FOV = 48
CAMERA_HORIZONTAL_FOV = 64
CAMERA_MIN_DEPTH_RANGE = 600
CAMERA_MAX_DEPTH_RANGE = 10000


def load_images(images_path: str, color_name: str, depth_name: str, mask_name: str):

    """
        Загрузка трех изображений: цвета, глубины, маски дороги

    :param images_path:
    :param color_name:
    :param depth_name:
    :param mask_name:
    :return:
    """

    color_image = cv2.imread(os.path.join(images_path, color_name))
    depth_image = cv2.imread(os.path.join(images_path, depth_name))
    mask_image = cv2.imread(os.path.join(images_path, mask_name))

    return color_image, depth_image, mask_image


def reverse_jet_colormap(colorized: np.ndarray):

    """
        Превращение изображения глубины, с примененным фильтром цветовой маски, обратно в изображения с закодированным
    расстоянием в каждом пикселе.

    :param colorized:
    :return:
    """

    depth_image = cv2.convertScaleAbs(colorized, alpha=0.5)
    depth_image = np.array(depth_image, dtype=np.uint16)
    depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY) * 64

    return depth_image


def combine_mask_and_depth(masked, depth):

    """
        Комбининация изображения глубины и маски дороги так, чтобы получилось изображение, где белый цвет - пересечение
    маски и глубины, черный цвет - остальное

    :param masked:
    :param depth:
    :return:
    """

    mask = np.copy(masked)
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    combined = cv2.bitwise_and(mask, depth)
    combined_binary = np.copy(combined)
    combined_binary[combined_binary > 15] = 255
    return combined_binary, combined


def find_edges(mask):

    """
        Нахождение крайних правой, левой, верхней и нижней точки на маске

    :param mask:
    :return:
    """

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    c = max(contours, key=cv2.contourArea)
    left = tuple(c[c[:, :, 0].argmin()][0])
    right = tuple(c[c[:, :, 0].argmax()][0])
    top = tuple(c[c[:, :, 1].argmin()][0])
    bottom = tuple(c[c[:, :, 1].argmax()][0])
    l_p = Point(left[0] + 10, left[1], "LEFT_POINT")
    t_p = Point(top[0], top[1] + 30, "TOP_POINT")
    r_p = Point(right[0] - 10, right[1], "RIGHT_POINT")
    b_p = Point(bottom[0], bottom[1] - 10, "BOTTOM_POINT")

    return l_p, t_p, r_p, b_p


def normalize_depth(depth: np.ndarray):

    """
        Исходное изображение глубины масштабируется из формата uint16 в uint8, отсекаются все пиксели, значения глубины
    которых находятся вне диапазона действия камеры глубины

    :param depth:
    :return:
    """

    normalized = np.copy(depth)
    # Отсечение всех пикселей с расстоянием больше чем максимальная глубина камеры путем их обнуления (черный цвет)
    normalized[normalized > CAMERA_MAX_DEPTH_RANGE] = 0
    # Отсечение всех пикселей с расстоянием меньше чем минимальная глубина камеры путем их обнуления (черный цвет)
    normalized[normalized < CAMERA_MIN_DEPTH_RANGE] = 0
    # Масштабирование расстояний в формат UINT8: 0...255
    normalized = normalized / 64
    normalized = np.array(normalized, np.uint8)
    return normalized


def binarize(image):
    """
        В черно-белом изображении заменяет все оттенки серого на белый цвет

    :param image:
    :return:
    """
    image[image > 0] = 255
    return image


def calc_real_points(depth_image: np.ndarray, points: list):
    return np.float32([[0, 0], [500, 100], [700, 600], [400, 600]])


def process_images(color_image: np.ndarray, depth_image: np.ndarray):
    """


    :param color_image:
    :param depth_image:
    :return:
    """
    # Нормализация изображения глубины из uint16 в uint8
    depth_uint8 = normalize_depth(depth_image)
    # Пересечение маски и глубины, получение новой бинарной маски
    combined_binary_image, combined_image = combine_mask_and_depth(maskImage, depth_uint8)
    # Нахождение крайних правой, левой, верхней и нижней точки на маске
    edgePoints = find_edges(combined_binary_image)
    # Отмечаем область в метр перед камерой, как доступную для движения
    combined_binary_image[400:, :] = 255
    # Установка значений глубины в крайние точки по исходному изображению глубины
    for point in edgePoints:
        point.set_depth(depthImage)
    # Перевод списка точек на изображении в массив np.ndarray
    image_points_coordinates = np.float32([point.get_coord() for point in edgePoints])
    # Получение списка точек реальных координат в массив np.ndarray
    real_points_coordinates = calc_real_points(depthImage, image_points_coordinates)

    perspective_matrix = cv2.getPerspectiveTransform(image_points_coordinates, real_points_coordinates)
    occupancy_grid = cv2.warpPerspective(combined_binary_image, perspective_matrix, (800, 600), borderValue=127)

    return occupancy_grid


class Point(object):

    FONT = cv2.FONT_ITALIC
    FONT_COLOR = (255, 255, 255)
    INTERNAL_CIRCLE_COLOR = (88, 219, 255)
    EXTERNAL_CIRCLE_COLOR = (0, 0, 0)

    def __init__(self, x: int, y: int, name: str = None):
        self.x = x
        self.y = y
        self.name = name

        self.depth = None

    def get_coord(self):
        return [self.x, self.y]

    def draw(self, canvas: np.ndarray):
        cv2.circle(canvas, (self.x, self.y), 5, self.EXTERNAL_CIRCLE_COLOR, -1)
        cv2.circle(canvas, (self.x, self.y), 3, self.INTERNAL_CIRCLE_COLOR, -1)

        if self.name:
            if self.x + 30 > canvas.shape[1]:
                cv2.putText(canvas, self.name, (self.x - 100, self.y - 8), self.FONT, 0.5, self.FONT_COLOR, 1)
            else:
                cv2.putText(canvas, self.name, (self.x, self.y - 8), self.FONT, 0.5, self.FONT_COLOR, 1)

        return canvas

    def set_depth(self, depth: np.ndarray):
        self.depth = depth[self.y, self.x]


if __name__ == '__main__':

    # Загрузка изображений
    colorImage, colorizedDepthImage, maskImage = load_images(images_path=IMAGES_PATH,
                                                             color_name=COLOR_IMAGE_NAME,
                                                             depth_name=DEPTH_IMAGE_NAME,
                                                             mask_name=MASK_IMAGE_NAME)
    # Преобразование цветного изображения глубины в обычное
    depthImage = reverse_jet_colormap(colorizedDepthImage)
    # Обработка изображений
    process_images(colorImage, depthImage)
