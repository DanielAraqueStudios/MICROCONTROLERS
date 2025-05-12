from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QFont
import random

class MatrixRain(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.opacity = 0.3  # Valor inicial de opacidad
        self.font = QFont("Courier", 10)
        self.characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
        self.drops = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_drops)
        self.timer.start(50)

    def setOpacity(self, value):
        self.opacity = value
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setOpacity(self.opacity)
        painter.setFont(self.font)
        painter.fillRect(self.rect(), Qt.GlobalColor.black)

        for drop in self.drops:
            x, y, char = drop
            painter.drawText(x, y, char)

    def update_drops(self):
        if random.random() < 0.1:
            x = random.randint(0, self.width())
            char = random.choice(self.characters)
            self.drops.append([x, 0, char])

        new_drops = []
        for drop in self.drops:
            x, y, char = drop
            y += 5
            if y < self.height():
                new_drops.append([x, y, char])
        self.drops = new_drops
        self.update()