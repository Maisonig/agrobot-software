import math

from shapely.ops import nearest_points
from shapely.geometry import Point, Polygon, LineString
from scripts.util.utils import decart_to_polar, polar_to_decart


def find_start_cords(poly, pos, direction, width):
    """"""

    min_distance = float('inf')

    coords = list(poly.exterior.coords)  # Координаты внешнего контура
    for i in range(len(coords) - 1):  # Избегаем повторения первой координаты в конце
        p1 = Point(coords[i])
        p2 = Point(coords[i + 1])

        distance = p1.distance(pos) + p2.distance(pos)

        if distance < min_distance:
            min_distance = distance
            nearest_point = p1 if p1.distance(pos) <= p2.distance(pos) else p2

    for j in range(len(coords) - 1):
        if coords[j] == (nearest_point.x, nearest_point.y):
            point_index = j
    num_vertices = len(coords) - 1  # Вычитаем 1, так как последняя точка повторяет первую

    next_index = (point_index + 1) % num_vertices
    prev_index = (point_index - 1 + num_vertices) % num_vertices # Обеспечиваем положительный индекс

    # Создаем объекты Point для следующей и предыдущей вершин
    next_point = Point(coords[next_index])
    prev_point = Point(coords[prev_index])
    next_rho, next_th = decart_to_polar(next_point.x - nearest_point.x, next_point.y - nearest_point.y)
    prev_rho, prev_th = decart_to_polar(prev_point.x - nearest_point.x , prev_point.y - nearest_point.y)
    angle_flg = 0
    if not is_angle_within_range(next_th, direction, math.pi / 4):
        # offset = next_point
        offset_th = next_th
        angle_flg += 1
    if not is_angle_within_range(prev_th, direction, math.pi / 4):
        # offset = prev_point
        offset_th = prev_th
        angle_flg += 1
    if angle_flg == 2:
        if not is_angle_within_range(next_th, direction + math.pi, math.pi / 4):
            # offset = next_point
            offset_th = next_th
        if not is_angle_within_range(prev_th, direction + math.pi, math.pi / 4):
            # offset = prev_point
            offset_th = prev_th

    try:
        # r, t = decart_to_polar(offset.x - nearest_point.x, offset.y - nearest_point.y)
        x, y = polar_to_decart(width / 2, offset_th)
        x, y, th, t = nearest_point.x + x, nearest_point.y + y, normalize_angle(offset_th), offset_th
        for a in [direction + math.pi / 2, direction - math.pi / 2]:
            if is_angle_within_range(t, a, math.pi / 2):
                t = a
        for a in [direction, direction + math.pi]:
            if is_angle_within_range(th, a, math.pi / 2):
                th = a
        return x, y, th, t
    except Exception:
        print("Can't build trajectory. Please set direction collinear to closest border")
        return None



def is_angle_within_range(next_th, th, tolerance=math.pi / 2):
    """
    Проверяет, находится ли угол next_th в пределах угла th +- tolerance.

    Args:
    next_th: Угол, который нужно проверить (в радианах).
    th: Центр диапазона (в радианах).
    tolerance: Допустимое отклонение от th (в радианах).

    Returns:
    True, если next_th находится в пределах th +- tolerance, иначе False.
    """

    # Нормализуем углы в пределах [-pi, pi]
    next_th = normalize_angle(next_th)
    th = normalize_angle(th)

    lower_bound = th - tolerance
    upper_bound = th + tolerance

    # Обрабатываем случай, когда диапазон пересекает границу [-pi, pi]
    if lower_bound < -math.pi:
        return next_th >= lower_bound + 2 * math.pi or next_th <= upper_bound
    elif upper_bound > math.pi:
        return next_th <= upper_bound - 2 * math.pi or next_th >= lower_bound
    else:
        return lower_bound <= next_th <= upper_bound


def normalize_angle(angle):
    """Нормализует угол в пределах [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def find_nearest_edge(polygon, point):
    """
    Находит ближайшую грань полигона к заданной точке.

    Args:
        polygon: Объект Polygon из библиотеки shapely.
        point: Объект Point из библиотеки shapely.

    Returns:
        Кортеж, содержащий:
            - Объект LineString, представляющий ближайшую грань.
            - Расстояние от точки до этой грани.
            - Точку на грани, ближайшую к исходной точке.
            Или None, если входные данные неверны.
    """

    if not isinstance(polygon, Polygon):
        print("Ошибка: polygon должен быть объектом Polygon.")
        return None
    if not isinstance(point, Point):
        print("Ошибка: point должен быть объектом Point.")
        return None

    # 1. Находим ближайшую точку на границе полигона
    nearest_point_on_boundary = nearest_points(polygon.boundary, point)[0]

    # 2. Находим ближайший отрезок (грань) полигона к этой точке
    coords = list(polygon.exterior.coords)  # Получаем координаты вершин полигона
    min_distance = float('inf')
    nearest_edge = None
    nearest_point_on_edge = None

    for i in range(len(coords) - 1):  # Перебираем отрезки
        p1 = Point(coords[i])
        p2 = Point(coords[i + 1])
        edge = LineString([p1, p2])
        distance = point.distance(edge) # Расстояние от точки до отрезка

        if distance < min_distance:
            min_distance = distance
            nearest_edge = edge
            nearest_point_on_edge = edge.interpolate(edge.project(point))  # Точка на грани, ближайшая к исходной точке

    if nearest_edge is None:
        return None

    return nearest_edge, min_distance, nearest_point_on_edge


def find_parallel_trajectory(poly, x0, y0, th0, lines_dir, width):
    """
    Строит траекторию параллельных линий по всему полигона со стартовой точки

    Args:
    poly: Полигон поля (shapely.Polygon).
    x0: Координата по оси х стартовой точки, должна быть в пределах полигона или на его грани.
    y0: Координата по оси у стартовой точки, должна быть в пределах полигона или на его грани.
    th0: Угол относительно глобальной оси координат робота, вдоль которого строятся параллельные линии.
    lines_dir: Угол относительно глобальной оси координат робота, по которому происходит смещение параллельных линий
    width: Расстояние между двумя соседними параллельными линиями


    Returns:
    list[tuple[float]] список координат x, y траектории
    """
    zero_length = 10000
    # Добавляем первую точку в список с траекторией
    points = [(x0, y0)]

    # while polygon.contains(Point(int(x0), int(y0))) or polygon.touches(Point(int(x0), int(y0))):
    for d in range(1000):
        # Строим луч условно бесконечной длины по направлению рядов, который точно пересечет полигон.
        # Находим координаты конца луча
        x1, y1 = polar_to_decart(zero_length, th0)
        # Преобразуем точки начала и конца луча в LineString из Shapely
        line = LineString(((x0, y0), (x1 + x0, y1 + y0)))
        # Находим точку пересечения луча с полигоном и обрезаем линию в этой точке
        line = poly.intersection(line)
        if line.is_empty:
            return LineString(points[:-1])
        # Добавляем точку пересечения в список траектории
        x1, y1 = list(line.coords)[-1]
        points.append((x1, y1))
        # Определяем ближайшую грань полигона к этой точке
        edge = find_nearest_edge(poly, Point((x1, y1)))[0].xy
        angle = decart_to_polar(edge[0][1] - edge[0][0], edge[1][1] - edge[1][0])[1]
        dir1 , dir2 = normalize_angle(angle + math.pi), normalize_angle(angle)
        for d in [dir1, dir2]:
            if is_angle_within_range(d, lines_dir, math.pi / 2):
                dir_ = d
        if is_angle_within_range(lines_dir, 0, math.pi / 2 - 0.034) or is_angle_within_range(lines_dir, math.pi, math.pi / 2 - 0.034):
            c = width / math.cos(dir_)
        if is_angle_within_range(lines_dir, math.pi / 2, math.pi / 2 - 0.034) or is_angle_within_range(lines_dir, 1.5 * math.pi, math.pi / 2 - 0.034):
            c = width / math.sin(dir_)
        x0, y0 = polar_to_decart(abs(c), dir_)
        x0, y0 = x0 + x1, y0 + y1
        th0 = normalize_angle(th0 + math.pi)
        points.append((x0, y0))
    line = LineString(points[:])
    return line


if __name__ == '__main__':

    # Пример полигона (задается в метрах)
    polygon_coords = [
        (0, 0),
        (50, 0),
        (50, 30),
        (0, 50),
    ]


    polygon = Polygon(polygon_coords)
    current_position = Point(-15, 0)

    lines_angle = 1.57
    lines_width = 3.

    if not polygon.is_valid:
        print("Некорректный полигон.")
        exit()


    offset_x, offset_y, offset_th, parallel_dir = find_start_cords(polygon, current_position, lines_angle, lines_width)
    traj = find_parallel_trajectory(polygon, offset_x, offset_y, offset_th, parallel_dir, lines_width)


    # Визуализация (требуется matplotlib)
    import matplotlib.pyplot as plt

    # Рисуем полигон
    x, y = polygon.exterior.xy
    plt.plot(x, y, 'k-', label='Полигон')
    try:
        x, y = traj.coords.xy
        plt.plot(x, y, 'g-', label='Траектория')
    except:
        pass

    # Рисуем точки
    plt.plot(current_position.x, current_position.y, 'bo', label='Текущее положение робота')
    plt.plot(offset_x, offset_y, 'ro', label='Стартовая точка')

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Траектория параллельного вождения")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
