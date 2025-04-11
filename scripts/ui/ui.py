import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout
from PyQt5.QtCore import QRect, QDate, QTime
from screeninfo import get_monitors

from scripts.ui.indicators import BatteryIndicatorWidget, DateTimeWidget, MainIndicators

for mon in get_monitors():
    if mon.is_primary:
        WINDOW_W = mon.width
        WINDOW_H = mon.height


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setGeometry(QRect(0, 0, WINDOW_W, WINDOW_H))

        self.mainIndicators = MainIndicators(self)

        self.show()

    def resizeEvent(self, event):
        if self.rect().width() < 1280:
            self.setGeometry(self.rect().x(), self.rect().y(), 1280, self.rect().height())
        if self.rect().height() < 720:
            self.setGeometry(self.rect().x(), self.rect().y(), self.rect().width(), 720)
        self.mainIndicators.update(self)



app = QApplication(sys.argv)
application = MainWindow()
application.show()

sys.exit(app.exec())
