#!/usr/bin/env python
import cv2
import numpy as np
import os
import glob
import pyrealsense2 as rs


def get_frames():
    run = True
    pipeline = rs.pipeline()  # <- Объект pipeline содержит методы для взаимодействия с потоком
    config = rs.config()  # <- Дополнительный объект для хранения настроек потока
    colorizer = rs.colorizer()  # <- Пригодится для отрисовки цветной карты глубины
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    cv2.namedWindow("RealSense object detection", cv2.WINDOW_AUTOSIZE)

    frame_num = 0
    while run:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RealSense object detection", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite(f"./calibration_{frame_num}.jpg", color_image)
            frame_num += 1
            if frame_num > 10:
                exit()


def get_params():
    # Определение размеров шахматной доски
    CHECKERBOARD = (8, 8)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Создание вектора для хранения векторов трехмерных точек для каждого изображения шахматной доски
    objpoints = []
    # Создание вектора для хранения векторов 2D точек для каждого изображения шахматной доски
    imgpoints = []
    # Определение мировых координат для 3D точек
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None
    # Извлечение пути отдельного изображения, хранящегося в данном каталоге
    images = glob.glob('./calibration_*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        scale = 80
        gray[gray > scale] = 255
        gray[gray < scale] = 0
        # Найти углы шахматной доски
        # Если на изображении найдено нужное количество углов, тогда ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        """
        Если желаемый номер угла обнаружен,
        уточняем координаты пикселей и отображаем
        их на изображениях шахматной доски
        """
        if ret == True:
            objpoints.append(objp)
            # уточнение координат пикселей для заданных 2d точек.
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            imgpoints.append(corners2)
            # Нарисовать и отобразить углы
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        cv2.imshow('img', img)
        cv2.waitKey(0)
    cv2.destroyAllWindows()
    h, w = img.shape[:2]
    """
    Выполнение калибровки камеры с помощью
    Передача значения известных трехмерных точек (объектов)
    и соответствующие пиксельные координаты
    обнаруженные углы (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    print("rvecs : \n")
    print(rvecs)
    print("tvecs : \n")
    print(tvecs)


while 1:
    a = input()
    if a == "f":
        get_frames()
        break
    if a == "g":
        get_params()
        break
