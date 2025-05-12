import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QPalette, QColor
import pyqtgraph as pg
import numpy as np

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
        
        # Serial configuration widget
        serial_widget = QWidget()
        serial_layout = QHBoxLayout(serial_widget)
        
        # COM port selection
        self.port_combo = QComboBox()
        self.update_ports()
        serial_layout.addWidget(QLabel("Port:"))
        serial_layout.addWidget(self.port_combo)
        
        # USB0 direct connect button
        self.usb0_button = QPushButton("Conectar por Cable (USB0)")
        self.usb0_button.clicked.connect(self.connect_usb0)
        serial_layout.addWidget(self.usb0_button)
        
        # Baud rate (fixed at 9600)
        baud_label = QLabel("Baud Rate: 9600")
        serial_layout.addWidget(baud_label)
        
        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        serial_layout.addWidget(self.connect_button)
        
        # Dark Mode button
        self.dark_mode_button = QPushButton("Dark Mode")
        self.dark_mode_button.clicked.connect(self.toggle_dark_mode)
        serial_layout.addWidget(self.dark_mode_button)
        
        layout.addWidget(serial_widget)
        
        # Graphs widget
        graphs_widget = QWidget()
        graphs_layout = QHBoxLayout(graphs_widget)
        
        # ADC plots
        self.adc_plot1 = self.create_plot("ADC1 Voltage", "Time", "Voltage (V)")
        self.adc_plot2 = self.create_plot("ADC2 Voltage", "Time", "Voltage (V)")
        self.freq_plot = self.create_plot("Frequency", "Time", "Frequency (Hz)")
        
        plots_layout = QVBoxLayout()
        plots_layout.addWidget(self.adc_plot1)
        plots_layout.addWidget(self.adc_plot2)
        plots_layout.addWidget(self.freq_plot)
        graphs_layout.addLayout(plots_layout)
        
        # ADC3 percentage gauge
        self.gauge_widget = self.create_gauge()
        graphs_layout.addWidget(self.gauge_widget)
        
        layout.addWidget(graphs_widget)
        
        # Data arrays
        self.timestamps = np.linspace(0, 100, 100)
        self.adc1_data = np.zeros(100)
        self.adc2_data = np.zeros(100)
        self.freq_data = np.zeros(100)
        
        # Serial port and timer setup
        self.serial_port = None
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        
    def toggle_dark_mode(self):
        self.dark_mode = not self.dark_mode
        # Update application palette
        app = QApplication.instance()
        palette = QPalette()
        
        if self.dark_mode:
            self.dark_mode_button.setText("Light Mode")
            palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
            palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
            palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
            palette.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
            palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
            palette.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
            
            # Update plots background
            self.adc_plot1.setBackground('k')
            self.adc_plot2.setBackground('k')
            self.freq_plot.setBackground('k')
            self.gauge_widget.setBackground('k')
            
        else:
            self.dark_mode_button.setText("Dark Mode")
            palette.setColor(QPalette.ColorRole.Window, QColor(255, 255, 255))
            palette.setColor(QPalette.ColorRole.WindowText, QColor(0, 0, 0))
            palette.setColor(QPalette.ColorRole.Base, QColor(255, 255, 255))
            palette.setColor(QPalette.ColorRole.Text, QColor(0, 0, 0))
            palette.setColor(QPalette.ColorRole.Button, QColor(255, 255, 255))
            palette.setColor(QPalette.ColorRole.ButtonText, QColor(0, 0, 0))
            
            # Update plots background
            self.adc_plot1.setBackground('w')
            self.adc_plot2.setBackground('w')
            self.freq_plot.setBackground('w')
            self.gauge_widget.setBackground('w')
        
        app.setPalette(palette)
        
    def create_plot(self, title, x_label, y_label):
        plot = pg.PlotWidget()
        plot.setBackground('w')
        plot.setTitle(title, color='k' if not self.dark_mode else 'w')
        plot.setLabel('left', y_label)
        plot.setLabel('bottom', x_label)
        plot.showGrid(x=True, y=True)
        return plot
        
    def create_gauge(self):
        gauge = pg.PlotWidget()
        gauge.setBackground('w')
        gauge.setTitle("ADC3 Percentage")
        gauge.setRange(yRange=(0, 100))
        gauge.hideAxis('bottom')
        self.gauge_bar = pg.BarGraphItem(x=[0], height=[0], width=0.6, brush='b')
        gauge.addItem(self.gauge_bar)
        return gauge
        
    def update_ports(self):
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        
    def connect_usb0(self):
        if self.serial_port is None:
            try:
                self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
                self.usb0_button.setText("Desconectar USB0")
                self.connect_button.setEnabled(False)
                self.port_combo.setEnabled(False)
                self.update_timer.start(100)
            except Exception as e:
                print(f"Error connecting to USB0: {e}")
        else:
            self.serial_port.close()
            self.serial_port = None
            self.usb0_button.setText("Conectar por Cable (USB0)")
            self.connect_button.setEnabled(True)
            self.port_combo.setEnabled(True)
            self.update_timer.stop()

    def toggle_connection(self):
        if self.serial_port is None:
            try:
                port = self.port_combo.currentText()
                self.serial_port = serial.Serial(port, 9600)
                self.connect_button.setText("Disconnect")
                self.usb0_button.setEnabled(False)
                self.update_timer.start(100)
            except Exception as e:
                print(f"Error: {e}")
        else:
            self.serial_port.close()
            self.serial_port = None
            self.connect_button.setText("Connect")
            self.usb0_button.setEnabled(True)
            self.update_timer.stop()

    def update_plots(self):
        if self.serial_port and self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode().strip()
                if line.startswith("ADC1:"):
                    voltage = float(line.split(":")[1].replace("V", ""))
                    self.adc1_data = np.roll(self.adc1_data, -1)
                    self.adc1_data[-1] = voltage
                    self.adc_plot1.plot(self.timestamps, self.adc1_data, clear=True, pen='b')
                    
                elif line.startswith("ADC2:"):
                    voltage = float(line.split(":")[1].replace("V", ""))
                    self.adc2_data = np.roll(self.adc2_data, -1)
                    self.adc2_data[-1] = voltage
                    self.adc_plot2.plot(self.timestamps, self.adc2_data, clear=True, pen='r')
                    
                elif line.startswith("ADC3:"):
                    voltage = float(line.split(":")[1].replace("V", ""))
                    percentage = (voltage / 3.3) * 100
                    self.gauge_bar.setOpts(height=[percentage])
                    
                elif line.startswith("Freq:"):
                    freq = float(line.split(":")[1].replace("Hz", ""))
                    self.freq_data = np.roll(self.freq_data, -1)
                    self.freq_data[-1] = freq
                    self.freq_plot.plot(self.timestamps, self.freq_data, clear=True, pen='g')
                    
            except Exception as e:
                print(f"Error parsing data: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WeatherStation()
    window.show()
    sys.exit(app.exec())
