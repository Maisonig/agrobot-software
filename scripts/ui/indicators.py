import sys

from scripts.ui.style import BLACK, LIGHT_GRAY
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QRect, QDate, QTime, QTimer
from style import DARK_RED, DARK_GREEN, DARK_YELLOW, ORANGE
from PyQt5.QtWidgets import QWidget, QApplication, QMainWindow, QHBoxLayout, QLabel


class BatteryIndicatorWidget(QWidget):
    MIN_SIZE = QRect(0, 0, 80, 18)
    def __init__(self, parent, min_charge: int = 0, max_charge: int = 100):
        super().__init__(parent=parent)
        self._charge = 0
        self._minCharge = min_charge
        self._maxCharge = max_charge

        self._timer = QTimer()
        self._timer.timeout.connect(self.update)
        self._timer.start(600)
        self._tick = False
        self._label = QLabel(self)

    def get_charge(self) -> int:
        return self._charge

    def update_charge(self, charge: int):
        self._charge = charge

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        self._draw_battery(painter)

    def _draw_battery(self, painter: QPainter) -> None:

        if self._charge < self._minCharge:
            self._charge = self._minCharge
        if self._charge > self._maxCharge:
            self._charge = self._maxCharge

        rect = self.rect()
        pen_width = 2
        painter.setPen(QPen(BLACK, pen_width))

        battery_width = int(rect.width() * 0.35 - pen_width)
        battery_height = int(rect.height() - pen_width)

        charge_width = int((battery_width - 3 * pen_width) * self._charge * 0.01)
        charge_height = int(battery_height - 3 * pen_width)

        battery_rect = QRect(pen_width // 2,
                             pen_width // 2,
                             battery_width,
                             battery_height
                             )

        painter.drawRect(battery_rect)

        tip_rect = QRect(battery_width + pen_width,
                         battery_height // 2 - battery_height // 4,
                         int(battery_width * 0.04),
                         battery_height - battery_height // 4 - pen_width)

        painter.drawRect(tip_rect)


        if self._charge < 20:
            c = DARK_RED
        elif self._charge > 80:
            c = DARK_GREEN
        else:
            c = BLACK
        painter.setBrush(QBrush(c))
        painter.setPen(QPen(c))
        painter.setFont(QFont('Italic', battery_height - 3 * pen_width))
        painter.drawText(battery_width + int(battery_width * 0.04) + pen_width + 3, battery_height - pen_width, f'{self._charge}%')
        if self._tick and self._charge < 20:
            self._tick = False
            return None
        else:
            charge_rect = QRect(int(2 * pen_width),
                                int(2 * pen_width),
                                charge_width,
                                charge_height)
            painter.drawRect(charge_rect)
            self._tick = True
        painter.end()

class DateTimeWidget(QLabel):
    MIN_SIZE = QRect(0, 0, 200, 18)
    def __init__(self, parent):
        super().__init__(parent=parent)
        self._timer = QTimer()
        self._timer.timeout.connect(self._draw_datetime)
        self._timer.start(1000)

    def _draw_datetime(self) -> None:
        rect = self.rect()
        text_height = int(rect.height() - 8)

        self.setFont(QFont('Italic', text_height))
        d = QDate.currentDate().toString('dd.MM.yy')
        t = QTime.currentTime().toString('hh:mm:ss')
        self.setText(f'{d}   {t}')

class TextWidget(QWidget):
    MIN_SIZE = QRect(0, 0, 110, 18)
    def __init__(self, parent, text):
        super().__init__(parent=parent)
        self.widgetText = text

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        text_height = int(self.rect().height() - 11)
        painter.setFont(QFont('Italic', text_height))
        painter.drawText(3, self.rect().height() - 3, self.widgetText)

class SimpleStatusWidget(QWidget):
    MIN_SIZE = QRect(0, 0, 60, 18)

    OK = DARK_GREEN
    WARN = DARK_YELLOW
    ERROR = ORANGE
    FATAL = DARK_RED
    def __init__(self, parent: any, name: str):
        super().__init__(parent)
        self._statusName = name
        self._status = self.OK
        self._timer = QTimer()
        self._timer.timeout.connect(self.update)
        self._timer.start(1000)

    def get_status(self) -> int:
        return self._status

    def update_status(self, status: int):
        self._status = status

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        self._draw_status(painter)

    def _draw_status(self, painter: QPainter):
        rect = self.rect()
        pen_width = 3
        word_height = rect.height() - 3 * pen_width
        painter.setFont(QFont('Italic', word_height))
        painter.drawText(pen_width, rect.height() - int(pen_width * 1.5), f'{self._statusName}')
        word_len = len(self._statusName)
        painter.setPen(QPen(self._status))
        painter.setBrush(QBrush(self._status))
        circle_rect = QRect(int(word_height * 1.3 * word_len),
                            pen_width + pen_width // 2,
                            int(rect.height() / 1.5),
                            int(rect.height() / 1.5))
        painter.drawEllipse(circle_rect)
        painter.end()

class SystemStatusWidget(SimpleStatusWidget):
    MIN_SIZE = QRect(0, 0, 100, 18)
    def __init__(self, parent: any, name: str):
        super().__init__(parent=parent, name=name)
        self.tick = False

    def _draw_status(self, painter: QPainter):
        rect = self.rect()
        pen_width = 3
        word_height = int(rect.height() - 3 * pen_width)
        circle_diam = int(word_height * 1.3)
        painter.setPen(QPen(LIGHT_GRAY))
        painter.setBrush(QBrush(LIGHT_GRAY))
        painter.drawRect(rect)
        painter.setFont(QFont('Italic', word_height))
        painter.setPen(QPen(BLACK))
        painter.setBrush(QBrush(BLACK))
        painter.drawText(pen_width, rect.height() - int(pen_width * 1.5), f'{self._statusName}')
        painter.setPen(QPen(self._status))
        painter.setBrush(QBrush(self._status))
        if self._status == self.FATAL:
            if self.tick:
                circle_rect = QRect(rect.width() - circle_diam - pen_width,
                                    pen_width,
                                    circle_diam,
                                    circle_diam)
                painter.drawEllipse(circle_rect)
                self.tick = False
            else:
                self.tick = True
        else:
            circle_rect = QRect(rect.width() - circle_diam - pen_width,
                                pen_width,
                                circle_diam,
                                circle_diam)
            painter.drawEllipse(circle_rect)
        painter.end()


class MainIndicators:
    def __init__(self, parent: QMainWindow, min_window: QRect = QRect(0, 0, 1280, 720)):
        self.resolution = parent.rect()
        self.minResolution = min_window
        res_h_ratio = parent.rect().width() / min_window.width()
        res_v_ratio = parent.rect().height() / min_window.height()

        self.gnsI = SimpleStatusWidget(parent, 'GNS')
        self.gnsI.setGeometry(9, 3, int(SimpleStatusWidget.MIN_SIZE.width() * res_h_ratio),
                              int(SimpleStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self._4GI = SimpleStatusWidget(parent, '4G')
        self._4GI.setGeometry(3 + self.gnsI.rect().width(), 3, int(SimpleStatusWidget.MIN_SIZE.width() * res_h_ratio),
                              int(SimpleStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.batteryI = BatteryIndicatorWidget(parent)
        self.batteryI.setGeometry(parent.rect().width() - int(300 * res_h_ratio), 3, int(BatteryIndicatorWidget.MIN_SIZE.width() * res_h_ratio),
                                          int(SimpleStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.dateTimeI = DateTimeWidget(parent)
        self.dateTimeI.setGeometry(parent.rect().width() - int(200 * res_h_ratio), 3, int(DateTimeWidget.MIN_SIZE.width() * res_h_ratio),
                                                                       int(DateTimeWidget.MIN_SIZE.height() * res_v_ratio))

        self.systemL = TextWidget(parent, 'Systems Status:')
        self.systemL.setGeometry(9, parent.rect().height() - int(TextWidget.MIN_SIZE.height() * res_v_ratio) - 9,
                                 int(TextWidget.MIN_SIZE.width() * res_h_ratio),
                                 int(TextWidget.MIN_SIZE.height() * res_v_ratio))

        self.motorsI = SystemStatusWidget(parent, 'Motors')
        self.motorsI.setGeometry(int(self.systemL.rect().topRight().x() * res_h_ratio) + 9, self.systemL.rect().topRight().y() + self.systemL.y(),
                                 int(SystemStatusWidget.MIN_SIZE.width() * res_h_ratio),
                                 int(SystemStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.sensorsI = SystemStatusWidget(parent, 'Sensors')
        self.sensorsI.setGeometry(int(self.motorsI.x() + self.motorsI.width()) + 9, self.motorsI.rect().topRight().y() + self.motorsI.y(),
                                 int(SystemStatusWidget.MIN_SIZE.width() * res_h_ratio),
                                 int(SystemStatusWidget.MIN_SIZE.height() * res_v_ratio))


    def update(self, parent: QMainWindow, min_window: QRect = QRect(0, 0, 1280, 720)):
        res_h_ratio = parent.rect().width() / min_window.width()
        res_v_ratio = parent.rect().height() / min_window.height()
        self.gnsI.setGeometry(9, 3, int(SimpleStatusWidget.MIN_SIZE.width() * res_h_ratio),
                              int(SimpleStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self._4GI.setGeometry(3 + self.gnsI.rect().width(), 3, int(SimpleStatusWidget.MIN_SIZE.width() * res_h_ratio),
                              int(SimpleStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.batteryI.setGeometry(parent.rect().width() - int(300 * res_h_ratio), 3, int(BatteryIndicatorWidget.MIN_SIZE.width() * res_h_ratio),
                                          int(SimpleStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.dateTimeI.setGeometry(parent.rect().width() - int(200 * res_h_ratio), 3, int(DateTimeWidget.MIN_SIZE.width() * res_h_ratio),
                                                                       int(DateTimeWidget.MIN_SIZE.height() * res_v_ratio))

        self.systemL.setGeometry(9, parent.rect().height() - int(TextWidget.MIN_SIZE.height() * res_v_ratio) - 9,
                                 int(TextWidget.MIN_SIZE.width() * res_h_ratio),
                                 int(TextWidget.MIN_SIZE.height() * res_v_ratio))

        self.motorsI.setGeometry(int(self.systemL.x() + self.systemL.width()) + 9, self.systemL.rect().topRight().y() + self.systemL.y(),
                                 int(SystemStatusWidget.MIN_SIZE.width() * res_h_ratio),
                                 int(SystemStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.sensorsI.setGeometry(int(self.motorsI.x() + self.motorsI.width()) + 9, self.motorsI.rect().topRight().y() + self.motorsI.y(),
                                 int(SystemStatusWidget.MIN_SIZE.width() * res_h_ratio),
                                 int(SystemStatusWidget.MIN_SIZE.height() * res_v_ratio))

        self.sensorsI.update_status(SystemStatusWidget.FATAL)


def main():
    app = QApplication(sys.argv)
    wind = QMainWindow()
    wind.setGeometry(0, 0, 640, 480)

    l = QHBoxLayout()

    battery = BatteryIndicatorWidget(wind)
    battery.setGeometry(QRect(410, 5, 80, 18))
    battery.update_charge(5)
    l.addWidget(battery)

    date = DateTimeWidget(wind)
    date.setGeometry(490, 5, 180, 18)
    l.addWidget(date)

    gns = SimpleStatusWidget(wind, 'GNS')
    gns.setGeometry(0, 5, 50, 18)
    l.addWidget(gns)

    g4 = SimpleStatusWidget(wind, '4G')
    g4.setGeometry(50, 5, 50, 18)
    l.addWidget(g4)

    motors = SystemStatusWidget(wind, 'Motors')
    motors.setGeometry(100, 450, 110, 18)
    motors.update_status(motors.FATAL)
    l.addWidget(motors)

    sensors = SystemStatusWidget(wind, 'Sensors')
    sensors.setGeometry(215, 450, 110, 18)
    sensors.update_status(sensors.WARN)
    l.addWidget(sensors)

    temperature = SystemStatusWidget(wind, 'Temperature')
    temperature.setGeometry(330, 450, 110, 18)
    temperature.update_status(temperature.ERROR)
    l.addWidget(temperature)

    connection = SystemStatusWidget(wind, 'Connection')
    connection.setGeometry(445, 450, 110, 18)
    connection.update_status(connection.OK)
    l.addWidget(connection)

    wind.setLayout(l)
    wind.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()