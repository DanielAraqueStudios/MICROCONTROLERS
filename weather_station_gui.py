from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QPalette, QColor, QPixmap
import pyqtgraph as pg
import numpy as np
import os

class WeatherStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.dark_mode = False
        self.setWindowTitle("Weather Station Monitor")
        self.setGeometry(100, 100, 1200, 800)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Header layout with logo and controls
        header_widget = QWidget()
        header_layout = QHBoxLayout(header_widget)
        
        # Logo
        logo_label = QLabel()
        logo_path = os.path.join(os.path.dirname(__file__), "resources", "logo.png")
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            scaled_pixmap = pixmap.scaled(100, 100)  # Ajusta el tamaño según necesites
            logo_label.setPixmap(scaled_pixmap)
        header_layout.addWidget(logo_label)
        
        # Serial configuration widget
        serial_widget = QWidget()
        serial_layout = QHBoxLayout(serial_widget)
        
        # ...existing serial configuration code...
        
        header_layout.addWidget(serial_widget)
        header_layout.addStretch()  # Pushes everything to the left
        
        layout.addWidget(header_widget)
        
        # ...rest of existing code...

# ...existing code...