from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel, 
                           QDoubleSpinBox, QFrame, QGroupBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QPalette
import sys
import serial.tools.list_ports

class SignalGeneratorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Signal Generator Control")
        self.setMinimumSize(800, 600)
        self.serial = None
        
        # Establecer el tema oscuro
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QLabel {
                color: #ffffff;
                font-size: 12px;
            }
            QPushButton {
                background-color: #0d47a1;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1565c0;
            }
            QPushButton:pressed {
                background-color: #0a2472;
            }
            QPushButton:disabled {
                background-color: #263238;
                color: #546e7a;
            }
            QComboBox {
                background-color: #424242;
                color: white;
                border: 1px solid #666666;
                border-radius: 4px;
                padding: 5px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: url(down_arrow.png);
                width: 12px;
                height: 12px;
            }
            QDoubleSpinBox {
                background-color: #424242;
                color: white;
                border: 1px solid #666666;
                border-radius: 4px;
                padding: 5px;
            }
            QGroupBox {
                color: white;
                border: 2px solid #666666;
                border-radius: 6px;
                margin-top: 12px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
        """)

        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Panel de conexión
        connection_group = QGroupBox("Connection Settings")
        connection_layout = QHBoxLayout()
        
        # Selector de puerto COM
        self.port_combo = QComboBox()
        self.refresh_ports()
        
        # Botón de conexión
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        # Indicador de estado
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: #ff5252;")  # Rojo para desconectado
        
        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.status_label)
        connection_group.setLayout(connection_layout)
        
        # Configuración para DAC1 y DAC2
        dac_layout = QHBoxLayout()
        
        # DAC1 Controls
        self.create_dac_controls(dac_layout, "DAC1")
        
        # Separador vertical
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.VLine)
        separator.setStyleSheet("background-color: #666666;")
        dac_layout.addWidget(separator)
        
        # DAC2 Controls
        self.create_dac_controls(dac_layout, "DAC2")
        
        # Añadir todo al layout principal
        main_layout.addWidget(connection_group)
        main_layout.addLayout(dac_layout)
        
        # Timer para actualizar la lista de puertos
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(1000)  # Actualizar cada segundo

    def create_dac_controls(self, parent_layout, dac_name):
        dac_group = QGroupBox(dac_name)
        dac_layout = QVBoxLayout()
        
        # Crear diccionario para almacenar widgets
        if not hasattr(self, 'dac_widgets'):
            self.dac_widgets = {}
        
        # Selector de forma de onda
        wave_layout = QHBoxLayout()
        wave_layout.addWidget(QLabel("Waveform:"))
        wave_combo = QComboBox()
        wave_combo.setObjectName(f"{dac_name}_wave")
        wave_combo.addItems(["Sine", "Triangle", "Square", "Sawtooth"])
        wave_layout.addWidget(wave_combo)
        dac_layout.addLayout(wave_layout)
        self.dac_widgets[f"{dac_name}_wave"] = wave_combo
        
        # Control de frecuencia
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Frequency (Hz):"))
        freq_spin = QDoubleSpinBox()
        freq_spin.setObjectName(f"{dac_name}_freq")
        freq_spin.setRange(0.1, 10000.0)
        freq_spin.setValue(100.0)
        freq_spin.setDecimals(1)
        freq_layout.addWidget(freq_spin)
        dac_layout.addLayout(freq_layout)
        self.dac_widgets[f"{dac_name}_freq"] = freq_spin
        
        # Control de amplitud
        amp_layout = QHBoxLayout()
        amp_layout.addWidget(QLabel("Amplitude (V):"))
        amp_spin = QDoubleSpinBox()
        amp_spin.setObjectName(f"{dac_name}_amp")
        amp_spin.setRange(0.0, 3.3)
        amp_spin.setValue(1.5)
        amp_spin.setSingleStep(0.1)
        amp_layout.addWidget(amp_spin)
        dac_layout.addLayout(amp_layout)
        self.dac_widgets[f"{dac_name}_amp"] = amp_spin
        
        # Control de offset
        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Offset (V):"))
        offset_spin = QDoubleSpinBox()
        offset_spin.setObjectName(f"{dac_name}_offset")
        offset_spin.setRange(0.0, 3.3)
        offset_spin.setValue(1.5)
        offset_spin.setSingleStep(0.1)
        offset_layout.addWidget(offset_spin)
        dac_layout.addLayout(offset_layout)
        self.dac_widgets[f"{dac_name}_offset"] = offset_spin
        
        # Botón de aplicar cambios
        apply_button = QPushButton("Apply Changes")
        apply_button.clicked.connect(lambda: self.apply_changes(dac_name))
        dac_layout.addWidget(apply_button)
        
        dac_group.setLayout(dac_layout)
        parent_layout.addWidget(dac_group)

    def refresh_ports(self):
        """Actualizar lista de puertos COM disponibles"""
        current_port = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if current_port in ports:
            self.port_combo.setCurrentText(current_port)

    def toggle_connection(self):
        """Conectar/Desconectar del puerto serial"""
        if self.serial is None or not self.serial.is_open:
            try:
                self.serial = serial.Serial(self.port_combo.currentText(), 9600)
                self.connect_button.setText("Disconnect")
                self.status_label.setText("Status: Connected")
                self.status_label.setStyleSheet("color: #4caf50;")  # Verde para conectado
            except Exception as e:
                self.status_label.setText(f"Error: {str(e)}")
                self.status_label.setStyleSheet("color: #ff5252;")
        else:
            self.serial.close()
            self.serial = None
            self.connect_button.setText("Connect")
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("color: #ff5252;")

    def apply_changes(self, dac_name):
        """Enviar cambios al generador de señales"""
        if self.serial is None or not self.serial.is_open:
            return
            
        try:
            # Obtener valores usando el diccionario de widgets
            wave_type = self.dac_widgets[f"{dac_name}_wave"].currentIndex()
            frequency = self.dac_widgets[f"{dac_name}_freq"].value()
            amplitude = self.dac_widgets[f"{dac_name}_amp"].value()
            offset = self.dac_widgets[f"{dac_name}_offset"].value()
            
            dac_num = "1" if dac_name == "DAC1" else "2"
            
            # Enviar comandos
            commands = [
                f"{dac_num}W{wave_type}",
                f"{dac_num}F{frequency}",
                f"{dac_num}A{amplitude}",
                f"{dac_num}O{offset}"
            ]
            
            for cmd in commands:
                print(f"Sending: {cmd}")  # Debug print
                self.serial.write(f"{cmd}\r\n".encode())
                self.serial.flush()
            
            # Actualizar estado
            self.status_label.setText(f"Status: Commands sent to {dac_name}")
            self.status_label.setStyleSheet("color: #4caf50;")
            
        except Exception as e:
            print(f"Error: {str(e)}")
            self.status_label.setText(f"Error sending commands: {str(e)}")
            self.status_label.setStyleSheet("color: #ff5252;")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SignalGeneratorGUI()
    window.show()
    sys.exit(app.exec())
