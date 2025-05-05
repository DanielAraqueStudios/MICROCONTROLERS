import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel, QGroupBox)
from PyQt6.QtCore import QTimer
import pyqtgraph as pg
import numpy as np

class MPU6050Viewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Accelerometer Viewer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Variables para datos
        self.data_points = 200  # Puntos a mostrar
        self.times = np.zeros(self.data_points)
        self.acc_x = np.zeros(self.data_points)
        self.acc_y = np.zeros(self.data_points)
        self.acc_z = np.zeros(self.data_points)
        
        self.serial_port = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Grupo de conexión
        connection_group = QGroupBox("Connection Settings")
        connection_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.refresh_button = QPushButton("Refresh Ports")
        self.refresh_button.clicked.connect(self.refresh_ports)
        
        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.refresh_button)
        connection_group.setLayout(connection_layout)
        
        # Configuración de gráficas
        plot_widget = pg.GraphicsLayoutWidget()
        
        # Gráfica para aceleración
        self.acc_plot = plot_widget.addPlot(title="Acceleration vs Time")
        self.acc_plot.setLabel('left', "Acceleration", units='g')
        self.acc_plot.setLabel('bottom', "Time", units='s')
        self.acc_plot.addLegend()
        
        # Líneas para cada eje
        self.line_x = self.acc_plot.plot(pen='r', name='X-Axis')
        self.line_y = self.acc_plot.plot(pen='g', name='Y-Axis')
        self.line_z = self.acc_plot.plot(pen='b', name='Z-Axis')
        
        # Añadir widgets al layout principal
        layout.addWidget(connection_group)
        layout.addWidget(plot_widget)
        
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
            
    def toggle_connection(self):
        if self.serial_port is None:
            try:
                port = self.port_combo.currentText()
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.connect_button.setText("Disconnect")
                self.timer.start(50)  # Actualizar cada 50ms
            except Exception as e:
                print(f"Error connecting: {e}")
        else:
            self.timer.stop()
            self.serial_port.close()
            self.serial_port = None
            self.connect_button.setText("Connect")
            
    def update_plot(self):
        if self.serial_port and self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                data = line.split('\t')
                if len(data) >= 4:  # Asegurarse de que hay suficientes datos
                    # Actualizar datos
                    self.acc_x = np.roll(self.acc_x, -1)
                    self.acc_y = np.roll(self.acc_y, -1)
                    self.acc_z = np.roll(self.acc_z, -1)
                    
                    self.acc_x[-1] = float(data[1])
                    self.acc_y[-1] = float(data[2])
                    self.acc_z[-1] = float(data[3])
                    
                    self.times = np.roll(self.times, -1)
                    self.times[-1] = self.times[-2] + 0.05
                    
                    # Actualizar gráficas
                    self.line_x.setData(self.times, self.acc_x)
                    self.line_y.setData(self.times, self.acc_y)
                    self.line_z.setData(self.times, self.acc_z)
            except Exception as e:
                print(f"Error reading data: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Estilo moderno
    viewer = MPU6050Viewer()
    viewer.show()
    sys.exit(app.exec())
