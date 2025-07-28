from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg
import sys
import serial
import numpy as np
from serial.tools import list_ports  # Agregar esta importación

class SignalGeneratorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Generador de Señales Dual")
        self.setGeometry(100, 100, 1200, 800)
        
        # Configuración del puerto serial
        self.serial_port = None
        
        # Widget principal y layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        
        # Panel de control
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        
        # Crear paneles para cada canal
        self.create_channel_panel(control_layout, "Canal 1", 1)
        self.create_channel_panel(control_layout, "Canal 2", 2)
        
        # Panel de conexión
        connection_panel = QHBoxLayout()
        
        # Selector de puerto COM
        self.port_combo = QComboBox()
        self.update_com_ports()
        connection_panel.addWidget(QLabel("Puerto:"))
        connection_panel.addWidget(self.port_combo)
        
        # Botón para actualizar puertos
        refresh_button = QPushButton("⟳")
        refresh_button.setMaximumWidth(30)
        refresh_button.clicked.connect(self.update_com_ports)
        connection_panel.addWidget(refresh_button)
        
        # Botón de conexión
        self.connect_button = QPushButton("Conectar")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_panel.addWidget(self.connect_button)
        
        # Agregar panel de conexión al layout principal
        connection_widget = QWidget()
        connection_widget.setLayout(connection_panel)
        control_layout.addWidget(connection_widget)
        
        # Panel de gráficos
        plot_panel = QWidget()
        plot_layout = QVBoxLayout(plot_panel)
        
        # Gráficos para cada canal
        self.plot_widget1 = pg.PlotWidget(title="Canal 1")
        self.plot_widget2 = pg.PlotWidget(title="Canal 2")
        plot_layout.addWidget(self.plot_widget1)
        plot_layout.addWidget(self.plot_widget2)
        
        # Agregar paneles al layout principal
        layout.addWidget(control_panel, 1)
        layout.addWidget(plot_panel, 2)
        
        # Timer para actualización de gráficos
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)  # Actualizar cada 100ms

    def create_channel_panel(self, parent_layout, title, channel):
        group = QGroupBox(title)
        layout = QVBoxLayout()
        
        # Selector de forma de onda
        wave_combo = QComboBox()
        wave_combo.addItems(["Senoidal", "Cuadrada", "Sierra", "Pulso"])
        layout.addWidget(QLabel("Forma de onda:"))
        layout.addWidget(wave_combo)
        
        # Control de frecuencia
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Frecuencia (Hz):"))
        freq_spin = QDoubleSpinBox()
        freq_spin.setRange(0.1, 1000000)
        freq_spin.setValue(1000)
        freq_layout.addWidget(freq_spin)
        layout.addLayout(freq_layout)
        
        # Control de amplitud
        amp_layout = QHBoxLayout()
        amp_layout.addWidget(QLabel("Amplitud (V):"))
        amp_spin = QDoubleSpinBox()
        amp_spin.setRange(0, 3.3)
        amp_spin.setValue(2)
        amp_layout.addWidget(amp_spin)
        layout.addLayout(amp_layout)
        
        # Control de offset
        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Offset (V):"))
        offset_spin = QDoubleSpinBox()
        offset_spin.setRange(0, 3.3)
        offset_spin.setValue(1.65)
        offset_layout.addWidget(offset_spin)
        layout.addLayout(offset_layout)
        
        # Botón de aplicar
        apply_button = QPushButton("Aplicar")
        apply_button.clicked.connect(lambda: self.apply_changes(channel))
        layout.addWidget(apply_button)
        
        group.setLayout(layout)
        parent_layout.addWidget(group)
        
        # Guardar referencias a los controles
        setattr(self, f"wave_combo_{channel}", wave_combo)
        setattr(self, f"freq_spin_{channel}", freq_spin)
        setattr(self, f"amp_spin_{channel}", amp_spin)
        setattr(self, f"offset_spin_{channel}", offset_spin)

    def update_com_ports(self):
        """Actualiza la lista de puertos COM disponibles"""
        self.port_combo.clear()
        ports = list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No hay puertos disponibles")

    def toggle_connection(self):
        if self.serial_port is None:
            try:
                port = self.port_combo.currentText()
                if port == "No hay puertos disponibles":
                    raise serial.SerialException("No hay puertos disponibles")
                    
                self.serial_port = serial.Serial(port, 115200)
                self.connect_button.setText("Desconectar")
                self.port_combo.setEnabled(False)
            except serial.SerialException as e:
                QMessageBox.critical(self, "Error", f"No se pudo conectar al puerto serial: {str(e)}")
        else:
            self.serial_port.close()
            self.serial_port = None
            self.connect_button.setText("Conectar")
            self.port_combo.setEnabled(True)

    def apply_changes(self, channel):
        if self.serial_port is None:
            return
            
        wave_combo = getattr(self, f"wave_combo_{channel}")
        freq_spin = getattr(self, f"freq_spin_{channel}")
        amp_spin = getattr(self, f"amp_spin_{channel}")
        offset_spin = getattr(self, f"offset_spin_{channel}")
        
        # Crear comando para enviar al microcontrolador
        command = f"CH{channel}:"
        command += f"W{wave_combo.currentIndex() + 1}:"
        command += f"F{freq_spin.value()}:"
        command += f"A{amp_spin.value()}:"
        command += f"O{offset_spin.value()}\n"
        
        self.serial_port.write(command.encode())

    def update_plots(self):
        # Generar datos de ejemplo para la visualización
        # En una implementación real, estos datos vendrían del microcontrolador
        t = np.linspace(0, 0.001, 1000)
        
        # Canal 1
        wave_type1 = self.wave_combo_1.currentIndex()
        freq1 = self.freq_spin_1.value()
        amp1 = self.amp_spin_1.value()
        offset1 = self.offset_spin_1.value()
        
        # Canal 2
        wave_type2 = self.wave_combo_2.currentIndex()
        freq2 = self.freq_spin_2.value()
        amp2 = self.amp_spin_2.value()
        offset2 = self.offset_spin_2.value()
        
        # Actualizar gráficos
        self.plot_widget1.clear()
        self.plot_widget2.clear()
        
        # Generar y mostrar formas de onda
        self.plot_widget1.plot(t, self.generate_waveform(t, wave_type1, freq1, amp1, offset1))
        self.plot_widget2.plot(t, self.generate_waveform(t, wave_type2, freq2, amp2, offset2))

    def generate_waveform(self, t, wave_type, freq, amp, offset):
        if wave_type == 0:  # Senoidal
            return amp * np.sin(2 * np.pi * freq * t) + offset
        elif wave_type == 1:  # Cuadrada
            return amp * np.sign(np.sin(2 * np.pi * freq * t)) + offset
        elif wave_type == 2:  # Sierra
            return amp * (2 * (freq * t - np.floor(freq * t)) - 1) + offset
        else:  # Pulso
            return amp * (np.sin(2 * np.pi * freq * t) > 0.9).astype(float) + offset

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SignalGeneratorGUI()
    window.show()
    sys.exit(app.exec())
