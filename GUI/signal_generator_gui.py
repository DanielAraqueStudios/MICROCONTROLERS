from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QPushButton, QLabel, 
                           QDoubleSpinBox, QFrame, QGroupBox, QButtonGroup,
                           QGridLayout)  # Added QGridLayout here
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QColor, QPalette, QFont, QFontDatabase, QPainter
import sys
import serial.tools.list_ports
import random

class MatrixEffect(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.chars = ['0', '1']  # Caracteres Matrix
        self.columns = []  # Lista de columnas de caracteres
        self.setMinimumSize(800, 600)
        
        # Timer para la animación
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(100)  # Actualizar cada 100ms
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(0, 0, 0))
        
        # Configurar el color y la fuente
        painter.setFont(QFont('Courier', 10))
        
        width = self.width()
        height = self.height()
        column_width = 15
        
        # Inicializar columnas si es necesario
        if not self.columns:
            num_columns = width // column_width
            self.columns = [{'pos': -random.randint(0, height), 
                           'speed': random.randint(3, 15)} 
                          for _ in range(num_columns)]
        
        # Dibujar caracteres Matrix
        for i, column in enumerate(self.columns):
            x = i * column_width
            y = column['pos']
            speed = column['speed']
            
            # Dibujar la columna de caracteres
            for j in range(0, height, 20):
                y_pos = (y + j) % height
                
                # Variar la opacidad
                opacity = 255 - (j * 2)
                if opacity < 0:
                    opacity = 0
                    
                painter.setPen(QColor(0, 255, 0, opacity))
                if random.random() > 0.95:  # Cambiar caracteres aleatoriamente
                    painter.setPen(QColor(200, 255, 200, opacity))
                
                char = random.choice(self.chars)
                painter.drawText(x, y_pos, char)
            
            # Actualizar posición
            column['pos'] += speed
            if column['pos'] > height:
                column['pos'] = -random.randint(0, height)
                column['speed'] = random.randint(3, 15)

class SignalGeneratorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Professional Signal Generator")
        self.setMinimumSize(1024, 768)  # Tamaño más profesional
        self.serial = None
        
        # Initialize dac_widgets dictionary
        self.dac_widgets = {}
        
        # Configure fonts
        self.setup_fonts()
        
        # Crear y configurar el widget Matrix
        self.matrix_effect = MatrixEffect(self)
        self.matrix_effect.setGeometry(self.rect())
        self.matrix_effect.lower()
        
        # Actualizar el tema a estilo Matrix
        self.setStyleSheet("""
            QMainWindow {
                background-color: rgba(0, 0, 0, 180);
            }
            QLabel {
                color: #00ff00;
                background-color: transparent;
            }
            QPushButton {
                background-color: #002200;
                color: #00ff00;
                border: 2px solid #00ff00;
                border-radius: 5px;
                padding: 10px;
                min-width: 100px;
            }
            QPushButton:checked {
                background-color: #004400;
                border: 2px solid #00ff00;
            }
            QPushButton:hover {
                background-color: #003300;
            }
            QDoubleSpinBox {
                background-color: rgba(0, 20, 0, 180);
                color: #00ff00;
                border: 2px solid #00ff00;
                border-radius: 5px;
                padding: 5px;
                min-width: 150px;
            }
            QGroupBox {
                color: #00ff00;
                border: 2px solid #00ff00;
                border-radius: 10px;
                margin-top: 20px;
                padding: 15px;
                background-color: rgba(0, 0, 0, 150);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 20px;
                padding: 0 10px;
                background-color: rgba(0, 0, 0, 180);
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

    def setup_fonts(self):
        # Fuentes más profesionales
        self.fonts = {
            'title': QFont("Segoe UI", 24, QFont.Weight.Bold),
            'heading': QFont("Segoe UI", 16, QFont.Weight.DemiBold),
            'display': QFont("Digital-7", 20, QFont.Weight.Normal),  # Fuente tipo display
            'normal': QFont("Segoe UI", 12),
            'values': QFont("Consolas", 14, QFont.Weight.Bold)
        }
        QApplication.setFont(self.fonts['normal'])

    def create_dac_controls(self, parent_layout, dac_name):
        # Crear un contenedor principal para el DAC
        dac_container = QWidget()
        container_layout = QVBoxLayout(dac_container)
        
        # Título del DAC con estilo de display digital
        title_label = QLabel(dac_name)
        title_label.setFont(self.fonts['title'])
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setStyleSheet("color: #00ff00; padding: 10px;")
        container_layout.addWidget(title_label)
        
        # Panel de visualización
        display_panel = QGroupBox("Output")
        display_panel.setFont(self.fonts['heading'])
        display_layout = QGridLayout()
        
        # Visualización actual de valores
        wave_display = QLabel("Sine")
        freq_display = QLabel("1000.0 Hz")
        amp_display = QLabel("1.50 V")
        offset_display = QLabel("0.00 V")
        
        for label in [wave_display, freq_display, amp_display, offset_display]:
            label.setFont(self.fonts['display'])
            label.setStyleSheet("color: #00ff00; background-color: rgba(0,20,0,180);")
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
            label.setMinimumWidth(200)
        
        display_layout.addWidget(QLabel("Wave:"), 0, 0)
        display_layout.addWidget(wave_display, 0, 1)
        display_layout.addWidget(QLabel("Freq:"), 1, 0)
        display_layout.addWidget(freq_display, 1, 1)
        display_layout.addWidget(QLabel("Amp:"), 2, 0)
        display_layout.addWidget(amp_display, 2, 1)
        display_layout.addWidget(QLabel("Offset:"), 3, 0)
        display_layout.addWidget(offset_display, 3, 1)
        
        display_panel.setLayout(display_layout)
        container_layout.addWidget(display_panel)
        
        # Controles
        controls_panel = QGroupBox("Controls")
        controls_panel.setFont(self.fonts['heading'])
        controls_layout = QGridLayout()
        
        # Selector de forma de onda con botones grandes
        wave_buttons = QHBoxLayout()
        wave_types = ["Sine", "Square", "Triangle", "Sawtooth"]
        wave_group = QButtonGroup(self)
        wave_group.setObjectName(f"{dac_name}_wave")  # Add object name to button group
        
        for i, wave in enumerate(wave_types):
            btn = QPushButton(wave)
            btn.setCheckable(True)
            btn.setMinimumHeight(40)
            wave_group.addButton(btn, i)
            wave_buttons.addWidget(btn)
            if i == 0:  # Set first button as checked by default
                btn.setChecked(True)
        
        # Store the button group in dac_widgets
        self.dac_widgets[f"{dac_name}_wave"] = wave_group
        
        controls_layout.addLayout(wave_buttons, 0, 0, 1, 2)
        
        # Controles numéricos con diseño de instrumento
        params = [
            ("Frequency (Hz)", "freq", 0.1, 1000000, 1000.0, 1),
            ("Amplitude (V)", "amp", 0.0, 3.3, 1.5, 0.1),
            ("Offset (V)", "offset", 0.0, 3.3, 1.5, 0.1)
        ]
        
        for i, (label, name, min_val, max_val, default, step) in enumerate(params):
            # Label
            controls_layout.addWidget(QLabel(label), i+1, 0)
            
            # Spinner con estilo profesional
            spin = QDoubleSpinBox()
            spin.setObjectName(f"{dac_name}_{name}")
            spin.setRange(min_val, max_val)
            spin.setValue(default)
            spin.setSingleStep(step)
            spin.setFont(self.fonts['values'])
            spin.setMinimumHeight(35)
            spin.setAlignment(Qt.AlignmentFlag.AlignRight)
            controls_layout.addWidget(spin, i+1, 1)
            
            self.dac_widgets[f"{dac_name}_{name}"] = spin
        
        controls_panel.setLayout(controls_layout)
        container_layout.addWidget(controls_panel)
        
        # Botón de aplicar cambios
        apply_button = QPushButton("APPLY CHANGES")
        apply_button.setFont(self.fonts['heading'])
        apply_button.setMinimumHeight(50)
        apply_button.clicked.connect(lambda: self.apply_changes(dac_name))
        container_layout.addWidget(apply_button)
        
        parent_layout.addWidget(dac_container)

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
            # Get wave type from button group
            wave_group = self.dac_widgets[f"{dac_name}_wave"]
            wave_type = wave_group.checkedId()  # Get selected button ID
            if wave_type == -1:  # No button selected
                wave_type = 0  # Default to first wave type
            
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
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.matrix_effect.setGeometry(self.rect())
        
        # Ajustar tamaños de widgets basado en el nuevo tamaño
        width = self.width()
        height = self.height()
        
        # Aquí puedes añadir lógica para redimensionar widgets específicos
        # basado en el nuevo tamaño de la ventana

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SignalGeneratorGUI()
    window.show()
    sys.exit(app.exec())
