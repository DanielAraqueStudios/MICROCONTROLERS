import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel,
                           QFrame, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, QTime, QDateTime
from PyQt6.QtGui import QPalette, QColor, QPixmap, QFont
import pyqtgraph as pg
import numpy as np
import os
from matrix_rain import MatrixRain

class WeatherStation(QMainWindow):
    # Constante privada para el nombre
    __LEONARDO_NAME = "• Leonardo Montealegre - Aveces duerme"
    
    def __init__(self):
        super().__init__()
        self.dark_mode = True  # Iniciar en modo oscuro
        self.setWindowTitle("Weather Station Monitor")
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0a0a0a;
            }
            QPushButton {
                background-color: #1a1a1a;
                color: #00ff00;
                border: 1px solid #00ff00;
                border-radius: 5px;
                padding: 5px;
                font-family: 'Ubuntu Mono';
            }
            QPushButton:hover {
                background-color: #00ff00;
                color: #000000;
            }
            QLabel {
                color: #00ff00;
                font-family: 'Ubuntu Mono';
            }
            QComboBox {
                background-color: #1a1a1a;
                color: #00ff00;
                border: 1px solid #00ff00;
                border-radius: 5px;
                padding: 5px;
            }
        """)

        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Title Section
        title_label = QLabel("UNIVERSIDAD MILITAR NUEVA GRANADA")
        title_label.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #00ff00;
            font-family: 'Ubuntu Mono';
            background-color: #1a1a1a;
            padding: 10px;
            border: 1px solid #00ff00;
            border-radius: 5px;
        """)
        title_label.setFont(QFont('Ubuntu Bold', 20))
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title_label)
        
        # Header with image and team info
        header_widget = QWidget()
        header_layout = QHBoxLayout(header_widget)
        
        # Image section (col-lg-4)
        image_widget = QLabel()
        image_path = os.path.join(os.path.dirname(__file__), "resources", "stm32.png")
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            scaled_pixmap = pixmap.scaled(300, 300, Qt.AspectRatioMode.KeepAspectRatio)
            image_widget.setPixmap(scaled_pixmap)
        image_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_layout.addWidget(image_widget, 4)
        
        # Team info section (col-lg-8)
        team_widget = QWidget()
        team_layout = QVBoxLayout(team_widget)
        
        team_title = QLabel("Equipo de Desarrollo")
        team_title.setFont(QFont('Ubuntu Medium', 14))
        team_layout.addWidget(team_title)
        
        team_members = [
            "• Daniel García Araque - Ingeniero de Software",
            self.__LEONARDO_NAME,  # Usando la constante protegida
            "• Andrés Fonseca Neme - Ingeniero electrónico"
        ]
        
        for member in team_members:
            member_label = QLabel(member)
            member_label.setFont(QFont('Ubuntu', 16, QFont.Weight.Bold))
            member_label.setStyleSheet("""
                color: #00ff00;
                font-weight: bold;
                padding: 5px;
                background-color: rgba(26, 26, 26, 0.7);
                border-radius: 5px;
                margin: 2px;
            """)
            team_layout.addWidget(member_label)
            
        team_layout.addStretch()
        header_layout.addWidget(team_widget, 8)
        
        # Add header to main layout
        layout.addWidget(header_widget)
        
        # Separator line
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        layout.addWidget(line)
        
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
        
        # Bluetooth connect button
        self.bluetooth_button = QPushButton("Conectar Bluetooth")
        self.bluetooth_button.clicked.connect(self.connect_bluetooth)
        serial_layout.addWidget(self.bluetooth_button)
        
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
        
        # Clock widget
        self.clock_label = QLabel()
        self.clock_label.setFont(QFont('Ubuntu Mono', 24))  # Aumentado tamaño de fuente
        self.clock_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.clock_label.setStyleSheet("padding: 10px; border-radius: 10px;")
        layout.insertWidget(1, self.clock_label)  # Insert after title
        
        # Clock timer
        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self.update_clock)
        self.clock_timer.start(1000)  # Update every second
        self.update_clock()  # Initial update
        
        # Create all plots and gauges first
        self.adc_plot1 = self.create_plot("ADC1 Voltage", "Time", "Voltage (V)")
        self.adc_plot2 = self.create_plot("ADC2 Voltage", "Time", "Voltage (V)")
        self.gauge_widget, self.gauge_bar = self.create_gauge("ADC3 Percentage")
        self.freq_gauge, self.freq_bar = self.create_gauge("Humedad %")

        # Graphs widget
        graphs_widget = QWidget()
        graphs_layout = QHBoxLayout(graphs_widget)
        
        # ADC plots section (70% del ancho)
        plots_layout = QVBoxLayout()
        
        # ADC1 plot y valor
        adc1_container = QWidget()
        adc1_layout = QVBoxLayout(adc1_container)
        adc1_layout.addWidget(self.adc_plot1)
        self.adc1_value_label = QLabel("ADC1: 0.00V")
        self.adc1_value_label.setStyleSheet("""
            color: #00ff00;
            font-family: 'Ubuntu Mono';
            font-size: 16px;
            font-weight: bold;
            padding: 5px;
            background-color: #1a1a1a;
            border: 1px solid #00ff00;
            border-radius: 5px;
        """)
        self.adc1_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        adc1_layout.addWidget(self.adc1_value_label)
        plots_layout.addWidget(adc1_container)
        
        # ADC2 plot y valor
        adc2_container = QWidget()
        adc2_layout = QVBoxLayout(adc2_container)
        adc2_layout.addWidget(self.adc_plot2)
        self.adc2_value_label = QLabel("ADC2: 0.00V")
        self.adc2_value_label.setStyleSheet("""
            color: #00ff00;
            font-family: 'Ubuntu Mono';
            font-size: 16px;
            font-weight: bold;
            padding: 5px;
            background-color: #1a1a1a;
            border: 1px solid #00ff00;
            border-radius: 5px;
        """)
        self.adc2_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        adc2_layout.addWidget(self.adc2_value_label)
        plots_layout.addWidget(adc2_container)

        graphs_layout.addLayout(plots_layout, stretch=70)
        
        # Gauges layout con valores
        gauges_layout = QVBoxLayout()
        
        # ADC3 gauge y valor
        adc3_container = QWidget()
        adc3_layout = QVBoxLayout(adc3_container)
        adc3_layout.addWidget(self.gauge_widget)
        self.adc3_value_label = QLabel("ADC3: 0.00V")
        self.adc3_value_label.setStyleSheet("""
            color: #00ff00;
            font-family: 'Ubuntu Mono';
            font-size: 16px;
            font-weight: bold;
            padding: 5px;
            background-color: #1a1a1a;
            border: 1px solid #00ff00;
            border-radius: 5px;
        """)
        self.adc3_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        adc3_layout.addWidget(self.adc3_value_label)
        gauges_layout.addWidget(adc3_container)
        
        # Frequency gauge y valor
        freq_container = QWidget()
        freq_layout = QVBoxLayout(freq_container)
        freq_layout.addWidget(self.freq_gauge)
        self.freq_value_label = QLabel("humedad 0.0 Hz")
        self.freq_value_label.setStyleSheet("""
            color: #00ff00;
            font-family: 'Ubuntu Mono';
            font-size: 16px;
            font-weight: bold;
            padding: 5px;
            background-color: #1a1a1a;
            border: 1px solid #00ff00;
            border-radius: 5px;
        """)
        self.freq_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        freq_layout.addWidget(self.freq_value_label)
        gauges_layout.addWidget(freq_container)

        graphs_layout.addLayout(gauges_layout, stretch=30)
        
        layout.addWidget(graphs_widget)
        
        # Data arrays
        self.timestamps = np.linspace(0, 100, 100)
        self.adc1_data = np.zeros(100)
        self.adc2_data = np.zeros(100)
        
        # Configuración inicial de las gráficas
        self.adc1_curve = self.adc_plot1.plot(pen='b')
        # Cambiar ADC2 de barras a línea continua
        self.adc2_curve = self.adc_plot2.plot(pen={'color': '#ff0000', 'width': 2})

        # Serial port and timer setup
        self.serial_port = None
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        
        # Matrix Rain Animation
        self.matrix = MatrixRain(self)
        self.matrix.setGeometry(0, 0, self.width(), self.height())
        self.matrix.lower()  # Poner animación detrás de todo

        # Añadir buffer circular para temperatura
        self.temp_buffer = [0, 0, 0]  # Buffer de 3 elementos
        self.temp_buffer_index = 0

        # Añadir buffer y variables para el viento
        self.wind_buffer = [0] * 10  # Buffer más grande para el viento
        self.wind_index = 0
        self.prev_wind_speed = 0
        self.wind_change_rate = 0.1  # Factor de suavizado

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Actualizar tamaño de la animación Matrix
        self.matrix.setGeometry(0, 0, self.width(), self.height())

    def update_clock(self):
        current_time = QDateTime.currentDateTime()
        display_text = current_time.toString('dd/MM/yyyy  hh:mm:ss')
        self.clock_label.setText(display_text)
        self.clock_label.setStyleSheet("""
            color: #00ff00;
            background-color: #1a1a1a;
            padding: 10px;
            border: 1px solid #00ff00;
            border-radius: 5px;
            font-family: 'Ubuntu Mono';
            font-size: 24px;
            font-weight: bold;
        """)

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
            self.gauge_widget.setBackground('k')
            self.freq_gauge.setBackground('k')
            
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
            self.gauge_widget.setBackground('w')
            self.freq_gauge.setBackground('w')
        
        app.setPalette(palette)
        # Update clock color
        self.update_clock()

    def create_plot(self, title, x_label, y_label):
        plot = pg.PlotWidget()
        plot.setBackground('#0a0a0a')
        plot.setTitle(title, color='#00ff00')
        plot.setLabel('left', y_label, color='#00ff00')
        plot.setLabel('bottom', x_label, color='#00ff00')
        plot.showGrid(x=True, y=True, alpha=0.3)
        # Configurar rangos específicos para cada gráfica
        if title == "ADC1 Voltage":
            plot.addLegend()
            plot.setYRange(0, 100)  # Rango para temperatura en Celsius
            plot.setTitle("Temperature (°C)", color='#00ff00')
            plot.setLabel('left', "Temperature (°C)", color='#00ff00')
        elif title == "ADC2 Voltage":
            plot.addLegend()
            plot.setYRange(0, 200)  # Rango para velocidad del viento en km/h
            plot.setTitle("Wind Speed (km/h)", color='#00ff00')
            plot.setLabel('left', "Wind Speed (km/h)", color='#00ff00')
        return plot
        
    def create_gauge(self, title):
        gauge = pg.PlotWidget()
        gauge.setBackground('#0a0a0a')
        gauge.setTitle(title, color='#00ff00')
        # Modificamos el título del gauge de ADC3
        if title == "ADC3 Percentage":
            gauge.setTitle("Light Intensity (%)", color='#00ff00')
        gauge.setRange(yRange=(0, 100))
        gauge.hideAxis('bottom')
        # Hacer la barra más estrecha (width=0.3 en lugar de 0.6)
        bar = pg.BarGraphItem(x=[0], height=[0], width=0.3, brush='#00ff00')
        gauge.addItem(bar)
        # Ajustar el rango del eje X para centrar la barra
        gauge.setRange(xRange=(-0.5, 0.5))
        return gauge, bar

    def update_ports(self):
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        
    #change the bluethoot COM  port    
    def connect_bluetooth(self):
        if self.serial_port is None:
            try:
                self.serial_port = serial.Serial('/dev/rfcomm0 ', 9600)
                self.bluetooth_button.setText("Desconectar Bluetooth")
                self.connect_button.setEnabled(False)
                self.usb0_button.setEnabled(False)
                self.port_combo.setEnabled(False)
                self.update_timer.start(100)
            except Exception as e:
                print(f"Error connecting to Bluetooth: {e}")
        else:
            self.serial_port.close()
            self.serial_port = None
            self.bluetooth_button.setText("Conectar Bluetooth")
            self.connect_button.setEnabled(True)
            self.usb0_button.setEnabled(True)
            self.port_combo.setEnabled(True)
            self.update_timer.stop()

    def connect_usb0(self):
        if self.serial_port is None:
            try:
                self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
                self.usb0_button.setText("Desconectar USB0")
                self.connect_button.setEnabled(False)
                self.bluetooth_button.setEnabled(False)  # Deshabilitar botón Bluetooth
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
                print(f"Received: {line}")  # Debug

                def extract_number(text):
                    try:
                        value = text.split()[0].replace("V", "").replace("Hz", "")
                        return float(value)
                    except Exception as e:
                        print(f"Error extracting number from: {text}")
                        return 0.0

                # Procesamiento especial para ADC1 cuando viene en la misma línea que ADC2
                if "ADC1:" in line:
                    try:
                        adc1_part = line[line.find("ADC1:"):].split()[1]
                        voltage = float(adc1_part.replace("V", ""))
                        # Convertir voltaje a temperatura usando la ecuación T = 55.60*V + 0.367
                        temperature = 55.60 * voltage + 0.367
                        
                        # Actualizar buffer circular
                        self.temp_buffer[self.temp_buffer_index] = temperature
                        self.temp_buffer_index = (self.temp_buffer_index + 1) % 3
                        
                        # Calcular promedio
                        avg_temperature = sum(self.temp_buffer) / 3
                        avg_temperature=avg_temperature+13
                        
                        # Actualizar gráfica y label con el valor promediado
                        self.adc1_data = np.roll(self.adc1_data, -1)
                        self.adc1_data[-1] = avg_temperature
                        self.adc1_curve.setData(self.timestamps, self.adc1_data)
                        self.adc1_value_label.setText(f"Temperature: {avg_temperature:.1f}°C")
                        print(f"Debug - ADC1 voltage: {voltage:.2f}V, Avg Temp: {avg_temperature:.1f}°C")
                    except Exception as e:
                        print(f"Error processing ADC1: {e}")

                # Procesamiento normal para ADC2, ADC3 y Freq
                if line.startswith("ADC2:"):
                    try:
                        voltage = extract_number(line.split(":")[1])
                        # Convertir voltaje a velocidad del viento
                        target_wind_speed = 58.943 * voltage + 2.037
                        
                        # Actualizar buffer circular
                        self.wind_buffer[self.wind_index] = target_wind_speed
                        self.wind_index = (self.wind_index + 1) % 10
                        
                        # Calcular promedio móvil
                        avg_wind = sum(self.wind_buffer) / 10
                        
                        # Suavizar transición
                        if abs(avg_wind - self.prev_wind_speed) > 20:  # Si el cambio es muy brusco
                            # Hacer transición suave
                            wind_speed = self.prev_wind_speed + (avg_wind - self.prev_wind_speed) * self.wind_change_rate
                        else:
                            wind_speed = avg_wind
                        
                        self.prev_wind_speed = wind_speed
                        
                        # Actualizar gráfica y label
                        self.adc2_data = np.roll(self.adc2_data, -1)
                        self.adc2_data[-1] = wind_speed
                        self.adc2_curve.setData(self.timestamps, self.adc2_data)
                        self.adc2_value_label.setText(f"Wind: {wind_speed:.1f} km/h")
                        print(f"Debug - ADC2 voltage: {voltage:.2f}V, Wind: {wind_speed:.1f} km/h")
                    except Exception as e:
                        print(f"Error processing ADC2: {e}")
                    
                elif line.startswith("ADC3:"):
                    try:
                        voltage = extract_number(line.split(":")[1])
                        # Convertir voltaje a porcentaje de luz usando la ecuación: porcentaje_luz = -45.45*V + 100
                        light_percentage = -45.45 * voltage + 100
                        # Asegurar que el porcentaje esté entre 0 y 100
                        light_percentage = max(0, min(100, light_percentage))
                        self.gauge_bar.setOpts(height=[light_percentage])
                        self.adc3_value_label.setText(f"Light: {light_percentage:.1f}%")
                    except Exception as e:
                        print(f"Error processing ADC3: {e}")
                    
                elif line.startswith("Freq:"):
                    try:
                        freq = extract_number(line.split(":")[1])
                        freq_percentage = min((freq / 65000.0) * 100, 100)
                        self.freq_bar.setOpts(height=[freq_percentage])
                        self.freq_value_label.setText(f"Humedad: {freq:.1f} ")
                    except Exception as e:
                        print(f"Error processing Freq: {e}")
                    
            except Exception as e:
                print(f"Error parsing data: {e}")
                print(f"Problematic line: {line}")  # Debug

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WeatherStation()
    window.show()
    sys.exit(app.exec())
