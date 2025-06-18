from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QIcon, QFont, QPalette, QColor
import pyqtgraph as pg
import sys
import serial
import numpy as np
from serial.tools import list_ports
import json
import os

class StyleSheet:
    MAIN_STYLE = """
        QMainWindow {
            background-color: #2b2b2b;
        }
        QGroupBox {
            border: 2px solid #444444;
            border-radius: 5px;
            margin-top: 1em;
            padding-top: 10px;
            color: #ffffff;
        }
        QLabel {
            color: #ffffff;
        }
        QPushButton {
            background-color: #0d47a1;
            color: white;
            border: none;
            padding: 5px 15px;
            border-radius: 3px;
        }
        QPushButton:hover {
            background-color: #1565c0;
        }
        QPushButton:pressed {
            background-color: #0a3d91;
        }
        QComboBox {
            background-color: #424242;
            color: white;
            border: 1px solid #555555;
            padding: 5px;
            border-radius: 3px;
        }
        QSpinBox, QDoubleSpinBox {
            background-color: #424242;
            color: white;
            border: 1px solid #555555;
            padding: 5px;
            border-radius: 3px;
        }
    """

class ChannelPanel(QGroupBox):
    channelChanged = pyqtSignal(dict)
    
    def __init__(self, channel_num, parent=None):
        super().__init__(f"Canal {channel_num}", parent)
        self.channel_num = channel_num
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Wave Type Selection with icons
        wave_layout = QHBoxLayout()
        wave_layout.addWidget(QLabel("Forma de onda:"))
        self.wave_combo = QComboBox()
        self.wave_combo.addItems(["Senoidal", "Cuadrada", "Sierra", "Pulso"])
        wave_layout.addWidget(self.wave_combo)
        layout.addLayout(wave_layout)
        
        # Frequency control with slider
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Frecuencia (Hz):"))
        self.freq_spin = QDoubleSpinBox()
        self.freq_spin.setRange(0.1, 1000000)
        self.freq_spin.setValue(1000)
        self.freq_slider = QSlider(Qt.Orientation.Horizontal)
        self.freq_slider.setRange(0, 100)
        freq_layout.addWidget(self.freq_spin)
        freq_layout.addWidget(self.freq_slider)
        layout.addLayout(freq_layout)
        
        # Amplitude control with visual feedback
        amp_layout = QHBoxLayout()
        amp_layout.addWidget(QLabel("Amplitud (V):"))
        self.amp_spin = QDoubleSpinBox()
        self.amp_spin.setRange(0, 3.3)
        self.amp_spin.setValue(2)
        self.amp_progress = QProgressBar()
        self.amp_progress.setRange(0, 330)
        amp_layout.addWidget(self.amp_spin)
        amp_layout.addWidget(self.amp_progress)
        layout.addLayout(amp_layout)
        
        # Offset control
        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Offset (V):"))
        self.offset_spin = QDoubleSpinBox()
        self.offset_spin.setRange(0, 3.3)
        self.offset_spin.setValue(1.65)
        offset_layout.addWidget(self.offset_spin)
        layout.addLayout(offset_layout)
        
        # Status indicators
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Estado: Listo")
        self.status_indicator = QLabel("●")
        self.status_indicator.setStyleSheet("color: green")
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.status_indicator)
        layout.addLayout(status_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        self.apply_button = QPushButton("Aplicar")
        self.save_button = QPushButton("Guardar")
        self.load_button = QPushButton("Cargar")
        button_layout.addWidget(self.apply_button)
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.load_button)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        self.connect_signals()
        
    def connect_signals(self):
        self.freq_slider.valueChanged.connect(self.update_frequency)
        self.freq_spin.valueChanged.connect(lambda: self.freq_slider.setValue(int(self.freq_spin.value())))
        self.amp_spin.valueChanged.connect(lambda: self.amp_progress.setValue(int(self.amp_spin.value() * 100)))
        self.apply_button.clicked.connect(self.emit_changes)
        self.save_button.clicked.connect(self.save_settings)
        self.load_button.clicked.connect(self.load_settings)
        
    def update_frequency(self, value):
        self.freq_spin.setValue(value * 10000 / 100)
        
    def emit_changes(self):
        data = {
            'channel': self.channel_num,
            'wave_type': self.wave_combo.currentIndex(),
            'frequency': self.freq_spin.value(),
            'amplitude': self.amp_spin.value(),
            'offset': self.offset_spin.value()
        }
        self.channelChanged.emit(data)
        
    def save_settings(self):
        data = {
            'wave_type': self.wave_combo.currentIndex(),
            'frequency': self.freq_spin.value(),
            'amplitude': self.amp_spin.value(),
            'offset': self.offset_spin.value()
        }
        filename, _ = QFileDialog.getSaveFileName(self, 
            "Guardar Configuración",
            f"channel_{self.channel_num}_config.json",
            "JSON (*.json)")
        if filename:
            with open(filename, 'w') as f:
                json.dump(data, f)
                
    def load_settings(self):
        filename, _ = QFileDialog.getOpenFileName(self,
            "Cargar Configuración",
            "",
            "JSON (*.json)")
        if filename:
            with open(filename, 'r') as f:
                data = json.load(f)
                self.wave_combo.setCurrentIndex(data['wave_type'])
                self.freq_spin.setValue(data['frequency'])
                self.amp_spin.setValue(data['amplitude'])
                self.offset_spin.setValue(data['offset'])

class SignalGeneratorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Generador de Señales Dual")
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet(StyleSheet.MAIN_STYLE)
        
        # Central widget setup
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Control panel (left side)
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        
        # Connection setup panel
        self.setup_connection_panel(control_layout)
        
        # Channel panels
        self.channel1 = ChannelPanel(1)
        self.channel2 = ChannelPanel(2)
        self.channel1.channelChanged.connect(self.handle_channel_change)
        self.channel2.channelChanged.connect(self.handle_channel_change)
        control_layout.addWidget(self.channel1)
        control_layout.addWidget(self.channel2)
        
        # Graph panel (right side)
        graph_panel = QWidget()
        graph_layout = QVBoxLayout(graph_panel)
        
        # Setup plots
        self.setup_plots(graph_layout)
        
        # Add panels to main layout
        main_layout.addWidget(control_panel, 1)
        main_layout.addWidget(graph_panel, 2)
        
        # Setup status bar
        self.statusBar().showMessage("Listo")
        
        # Start update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(100)
        
        # Setup menu bar
        self.setup_menu()

    def setup_connection_panel(self, parent_layout):
        conn_group = QGroupBox("Conexión")
        layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.update_com_ports()
        
        refresh_btn = QPushButton("⟳")
        refresh_btn.setFixedWidth(30)
        refresh_btn.clicked.connect(self.update_com_ports)
        
        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        layout.addWidget(QLabel("Puerto:"))
        layout.addWidget(self.port_combo)
        layout.addWidget(refresh_btn)
        layout.addWidget(self.connect_btn)
        
        conn_group.setLayout(layout)
        parent_layout.addWidget(conn_group)

    def setup_plots(self, parent_layout):
        # Configure plot styling
        pg.setConfigOption('background', '#2b2b2b')
        pg.setConfigOption('foreground', 'w')
        
        self.plot1 = pg.PlotWidget(title="Canal 1")
        self.plot2 = pg.PlotWidget(title="Canal 2")
        
        self.plot1.setLabel('left', 'Voltaje', 'V')
        self.plot1.setLabel('bottom', 'Tiempo', 's')
        self.plot2.setLabel('left', 'Voltaje', 'V')
        self.plot2.setLabel('bottom', 'Tiempo', 's')
        
        parent_layout.addWidget(self.plot1)
        parent_layout.addWidget(self.plot2)

    def setup_menu(self):
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('Archivo')
        save_action = file_menu.addAction('Guardar Todo')
        save_action.triggered.connect(self.save_all_settings)
        load_action = file_menu.addAction('Cargar Todo')
        load_action.triggered.connect(self.load_all_settings)
        
        # Help menu
        help_menu = menubar.addMenu('Ayuda')
        about_action = help_menu.addAction('Acerca de')
        about_action.triggered.connect(self.show_about)

    # ... rest of the existing methods ...
    
    def handle_channel_change(self, data):
        if not hasattr(self, 'serial_port') or self.serial_port is None:
            QMessageBox.warning(self, "Error", "No hay conexión serial establecida")
            return
            
        try:
            # Create command with checksum
            command = f"CH{data['channel']}:"
            command += f"W{data['wave_type']}:"
            command += f"F{data['frequency']}:"
            command += f"A{data['amplitude']}:"
            command += f"O{data['offset']}"
            
            # Add checksum
            checksum = sum(command.encode()) % 256
            command += f":${checksum:02X}\n"
            
            # Send and wait for acknowledgment
            self.serial_port.write(command.encode())
            response = self.serial_port.readline().decode().strip()
            
            if response != "ACK":
                raise Exception("No se recibió confirmación del dispositivo")
                
            self.statusBar().showMessage(f"Cambios aplicados al Canal {data['channel']}")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error al enviar datos: {str(e)}")
            self.statusBar().showMessage("Error en la comunicación")

    def show_about(self):
        QMessageBox.about(self,
            "Acerca de Signal Generator",
            "Generador de Señales Dual\n"
            "Versión 1.0\n\n"
            "Desarrollado para el curso de Microcontroladores\n"
            "Universidad Militar Nueva Granada")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SignalGeneratorGUI()
    window.show()
    sys.exit(app.exec())
