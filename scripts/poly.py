import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.affinity import rotate
import json  # для сохранения в json

def generate_row_coverage_path(polygon_coords, sprayer_width, start_point, end_point=None):
    """
    Генерирует маршрут для покрытия поля рядами культуры с опрыскивателем,
    с возможностью старта и финиша за пределами полигона.

    Args:
        polygon_coords (list of tuples): Координаты вершин полигона.
        sprayer_width (float): Ширина опрыскивателя.
        start_point (tuple): Координаты точки старта.
        end_point (tuple, optional): Координаты точки финиша. Defaults to None.

    Returns:
        list of tuples: Координаты точек маршрута.
    """

    try:
        polygon = Polygon(polygon_coords)
        if not polygon.is_valid:
            print("Ошибка: Задан некорректный полигон.")
            return None

        # 1. Определяем направление рядков
        nearest_edge = find_nearest_edge(polygon, start_point)
        if not nearest_edge:
            print("Не удалось определить направление рядков.")
            return None

        # 2. Вычисляем угол
        angle = np.arctan2(nearest_edge[1][1] - nearest_edge[0][1], nearest_edge[1][0] - nearest_edge[0][0])
        angle_degrees = np.degrees(angle)

        # 3. Поворачиваем полигон, стартовую и конечную точки
        rotated_polygon = rotate(polygon, -angle_degrees, origin='centroid')
        rotated_start_point = rotate(Point(start_point), -angle_degrees, origin='centroid')
        if end_point:
            rotated_end_point = rotate(Point(end_point), -angle_degrees, origin='centroid')
        else:
            rotated_end_point = None

        # 4. Генерируем горизонтальные проходы
        bounds = rotated_polygon.bounds
        y_start = bounds[1]
        y_end = bounds[3]
        x_start = bounds[0]
        x_end = bounds[2]
        y_coords = np.arange(y_start, y_end, sprayer_width)
        paths = []
        going_right = True

        for y in y_coords:
            line = LineString([(x_start, y), (x_end, y)])
            intersection = rotated_polygon.intersection(line)

            if not intersection.is_empty:
                if intersection.geom_type == 'LineString':
                    coords = list(intersection.coords)
                    paths.append(coords)
                elif intersection.geom_type == 'MultiLineString':
                    for line_segment in intersection.geoms:
                        paths.append(list(line_segment.coords))
            going_right = not going_right


        # 5. Объединяем проходы в маршрут (зигзаг)
        route = []

        # Добавляем стартовую точку
        route.append((rotated_start_point.x, rotated_start_point.y))

        for i in range(len(paths)):
            if i % 2 == 0:
                for point in paths[i]:
                    route.append(point)
            else:
                for point in reversed(paths[i]):
                    route.append(point)
        # Добавляем конечную точку, если она есть

        if rotated_end_point:
            route.append((rotated_end_point.x, rotated_end_point.y))

        # 6. Поворачиваем маршрут обратно
        final_route = []
        for point in route:
            point_rotate = rotate(Point(point[0], point[1]), angle_degrees, origin='centroid')
            final_route.append((point_rotate.x, point_rotate.y))
        return final_route

    except Exception as e:
        print(f"Произошла ошибка: {e}")
        return None


def find_nearest_edge(polygon, point):
    """Находит ближайший отрезок (сторону) полигона к заданной точке."""
    min_distance = float('inf')
    nearest_edge = None
    for i in range(len(polygon.exterior.coords) - 1):
        p1 = polygon.exterior.coords[i]
        p2 = polygon.exterior.coords[i + 1]
        line = LineString([p1, p2])
        distance = line.distance(Point(point))
        if distance < min_distance:
            min_distance = distance
            nearest_edge = (p1, p2)
    return nearest_edge


def print_ascii_route_lines(polygon_coords, route, width=40, height=20):
    """
    Рисует полигон и траекторию движения робота линиями с помощью ASCII-графики.
    """
    if not route or len(route) < 2:
        print("Нечего рисовать (нужен маршрут хотя бы из двух точек)!")
        return

    min_x = min(p[0] for p in polygon_coords + route)
    max_x = max(p[0] for p in polygon_coords + route)
    min_y = min(p[1] for p in polygon_coords + route)
    max_y = max(p[1] for p in polygon_coords + route)

    x_scale = width / (max_x - min_x) if (max_x - min_x) > 0 else 1
    y_scale = height / (max_y - min_y) if (max_y - min_y) > 0 else 1

    canvas = [[' ' for _ in range(width)] for _ in range(height)]

    # Рисуем полигон (очень приблизительно)
    for i in range(len(polygon_coords) - 1):
        x1 = int((polygon_coords[i][0] - min_x) * x_scale)
        y1 = int((polygon_coords[i][1] - min_y) * y_scale)
        x2 = int((polygon_coords[i+1][0] - min_x) * x_scale)
        y2 = int((polygon_coords[i+1][1] - min_y) * y_scale)

        draw_line_ascii(canvas, x1, y1, x2, y2, '#', width, height)

    # Рисуем траекторию движения робота (линиями)
    for i in range(len(route) - 1):
        x1 = int((route[i][0] - min_x) * x_scale)
        y1 = int((route[i][1] - min_y) * y_scale)
        x2 = int((route[i+1][0] - min_x) * x_scale)
        y2 = int((route[i+1][1] - min_y) * y_scale)

        draw_line_ascii(canvas, x1, y1, x2, y2, '*', width, height)

    # Выводим ASCII-графику
    for row in canvas:
        print(''.join(row))

def draw_line_ascii(canvas, x1, y1, x2, y2, symbol, width, height):
    """Рисует линию на ASCII-холсте (упрощенно)"""
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    x = x1
    y = y1

    while True:
        if 0 <= x < width and 0 <= y < height:
            canvas[height - 1 - y][x] = symbol
        if x == x2 and y == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def save_route_to_json(polygon_coords, route, filename="route_data.json"):
    """
    Сохраняет координаты полигона и маршрута в JSON-файл.
    """
    data = {
        "polygon_coords": polygon_coords,
        "route": route
    }
    try:
        with open(filename, "w") as f:
            json.dump(data, f, indent=4)
        print(f"Данные сохранены в файл {filename}")
    except Exception as e:
        print(f"Ошибка при сохранении в файл: {e}")


if __name__ == '__main__':
    polygon_coords = [
        (0, 0),
        (10, 0),
        (10, 5),
        (5, 7),
        (0, 7),
    ]
    sprayer_width = 1.5
    start_point = (-1, -1)  # Точка старта вне полигона
    end_point = (0, 7)  # Точка финиша вне полигона

    route = generate_row_coverage_path(polygon_coords, sprayer_width, start_point, end_point)

    if route:
        print("ASCII-графика маршрута для опрыскивания (линиями):")
        print_ascii_route_lines(polygon_coords, route)
        save_route_to_json(polygon_coords, route)
    else:
        print("Не удалось сгенерировать маршрут.")
