from utils import *


def combine_mask_and_depth(masked: np.ndarray, depth: np.ndarray):

    """
    Комбининация изображения глубины и маски дороги так, чтобы получилось изображение, где белый цвет - пересечение
    маски и глубины, черный цвет - остальное

    :param masked:
    :param depth:
    :return:
    """

    mask = np.copy(masked)
    mask = np.array(mask, np.uint8)
    combined = cv2.bitwise_and(mask, depth)
    combined_binary = np.copy(combined)
    combined_binary[combined_binary > 15] = 255
    return combined_binary, combined


def find_edges(mask: np.ndarray):

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
    t_p = Point(top[0], top[1] + 60, "TOP_POINT")
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


def calc_real_points(points: tuple[Point, Point, Point, Point]) -> np.ndarray:
    # return np.float32([[0, 0], [500, 100], [700, 600], [400, 600]])
    """
    Нахождение истиных координат четырех точек - экстремумов на изображении глубины

    :param points:
    :return:
    """
    FX_DEPTH = 5.8262448167737955e+02
    FY_DEPTH = 5.8269103270988637e+02
    CX_DEPTH = 3.1304475870804731e+02
    CY_DEPTH = 2.3844389626620386e+02
    real = []
    max_x = 0

    max_y = 0
    for p in points:
        x = (p.x - CX_DEPTH) * p.depth / FX_DEPTH
        y = (p.y - CY_DEPTH) * p.depth / FY_DEPTH
        real.append([x, y])
        if x > max_x:
            max_x = x
        if y > max_y:
            max_y = y
        width = max_x * 2
        height = max_y
    d_real = []
    w_k = width / 640
    h_k = height / 480
    for i in range(4):
        d_real.append([real[i][0] / w_k, real[i][1] / h_k])

    d_real = np.array(d_real, np.float32)
    return d_real


def calc_homography(depth_image: np.ndarray, points: tuple[Point, Point, Point, Point], scale=10) -> np.ndarray:
    """
    Нахождение матрицы гомографии

    :param depth_image:
    :param scale:
    :param points:
    :return:
    """

    img_pts = []
    for pt in points:
        img_pts.append((pt.x, pt.y))
    real_pts = [
                (5, depth_image[img_pts[1][1], img_pts[1][0]] / 1000),
                (15, depth_image[img_pts[2][1], img_pts[2][0]] / 1000),
                (10.75, depth_image[img_pts[3][1], img_pts[3][0]] / 1000),
                (9.25, depth_image[img_pts[0][1], img_pts[0][0]] / 1000)
                ]

    for i in range(4):
        real_pts[i] = tuple(scale * elem for elem in real_pts.__getitem__(i))

    real_pts = np.array(real_pts)
    img_pts = np.array(img_pts)

    h, status = cv2.findHomography(img_pts, real_pts)

    return h


def process_images(depth_image: np.ndarray, mask_image: np.ndarray):
    """
    Преобразует 1 кадр с камер глубины и цвета в сетку препятствий с помощью стереокулярного подхода.

    :param mask_image:
    :param depth_image:
    :return:
    """

    # Нормализация изображения глубины из uint16 в uint8
    depth_uint8 = normalize_depth(depth_image)
    # Пересечение маски и глубины, получение новой бинарной маски
    combined_binary_image, combined_image = combine_mask_and_depth(mask_image, depth_uint8)
    # Нахождение крайних правой, левой, верхней и нижней точки на маске
    edge_points = find_edges(combined_binary_image)
    # Отмечаем область в метр перед камерой, как доступную для движения
    combined_binary_image[400:, :] = 255
    # Установка значений глубины в крайние точки по исходному изображению глубины
    for point in edge_points:
        point.set_depth(depth_image)
    # Перевод списка точек на изображении в массив np.ndarray
    image_points_coordinates = np.float32([point.get_coord() for point in edge_points])
    # Получение списка точек реальных координат в массив np.ndarray
    real_points_coordinates = calc_real_points(edge_points)

    homography = calc_homography(depth_image, edge_points, OC_SCALE)

    occupancy = cv2.warpPerspective(combined_binary_image, homography, oc_size, borderValue=(127, 127, 127))

    occupancy = np.array(occupancy, np.uint8)
    oc_shape = occupancy.shape
    occupancy[int(oc_shape[1] / 1.05):, :] = 127

    # perspective_matrix = cv2.getPerspectiveTransform(image_points_coordinates, real_points_coordinates)
    # occupancy_grid = cv2.warpPerspective(combined_binary_image, perspective_matrix, (1000, 1000), borderValue=127)

    return occupancy

