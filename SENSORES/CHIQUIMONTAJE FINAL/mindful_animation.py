from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QPainterPath
import math

class MindfulAnimation(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.angle = 0
        self.circles = []
        for i in range(5):
            self.circles.append({
                'radius': 50 + i * 30,
                'speed': 0.5 - i * 0.1,
                'opacity': 0.1,
                'color': QColor(0, 255, 100, 25)
            })
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(50)
        self.setStyleSheet("background: transparent;")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        center_x = self.width() / 2
        center_y = self.height() / 2

        for circle in self.circles:
            path = QPainterPath()
            radius = circle['radius']
            x = center_x + math.cos(self.angle * circle['speed']) * 20
            y = center_y + math.sin(self.angle * circle['speed']) * 20
            
            painter.setOpacity(circle['opacity'])
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(circle['color'])
            
            path.addEllipse(x - radius, y - radius, radius * 2, radius * 2)
            painter.drawPath(path)

        self.angle += 0.1
