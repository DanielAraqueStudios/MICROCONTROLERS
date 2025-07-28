from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import sys
import serial
import serial.tools.list_ports
import numpy as np

class WaveGeneratorPro(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("Wave Generator Pro")
        self.setMinimumSize(1200, 800)
        
        # Dark theme stylesheet
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e1e;
                color: #ffffff;
            }
            QGroupBox {
                border: 2px solid #404040;
                border-radius: 5px;
                padding: 10px;
                margin-top: 10px;
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
            QComboBox, QSpinBox, QDoubleSpinBox {
                background-color: #333333;
                border: 1px solid #404040;
                border-radius: 3px;
                padding: 5px;
                color: white;
            }
            QLabel {
                color: #ffffff;
                font-size: 12px;
            }
        """)
        
        # Main layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        
        # Control panel (left side)
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        
        # Connection settings
        connection_group = QGroupBox("Connection Settings")
        conn_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(self.connect_btn)
        connection_group.setLayout(conn_layout)
        control_layout.addWidget(connection_group)
        
        # Create channel controls
        self.create_channel_controls(control_layout, "DAC1", 1)
        self.create_channel_controls(control_layout, "DAC2", 2)
        
        # Plot area (right side)
        plot_panel = QWidget()
        plot_layout = QVBoxLayout(plot_panel)
        
        # Configure plots
        pg.setConfigOption('background', '#2d2d2d')
        pg.setConfigOption('foreground', 'w')
        
        self.plot1 = pg.PlotWidget(title="DAC1 Output")
        self.plot2 = pg.PlotWidget(title="DAC2 Output")
        plot_layout.addWidget(self.plot1)
        plot_layout.addWidget(self.plot2)
        
        # Add panels to main layout
        layout.addWidget(control_panel, 1)
        layout.addWidget(plot_panel, 2)
        
        # Setup update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)
        
    def create_channel_controls(self, parent_layout, name, channel):
        group = QGroupBox(f"{name} Controls")
        layout = QVBoxLayout()
        
        # Waveform selection
        wave_layout = QHBoxLayout()
        wave_layout.addWidget(QLabel("Waveform:"))
        wave_combo = QComboBox()
        wave_combo.addItems(["Sine", "Square", "Sawtooth", "Pulse"])
        wave_combo.setObjectName(f"wave_{channel}")
        wave_layout.addWidget(wave_combo)
        layout.addLayout(wave_layout)
        
        # Frequency control
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Frequency (Hz):"))
        freq_spin = QDoubleSpinBox()
        freq_spin.setObjectName(f"freq_{channel}")
        freq_spin.setRange(0.1, 1000000)
        freq_spin.setValue(1000)
        freq_layout.addWidget(freq_spin)
        layout.addLayout(freq_layout)
        
        # Amplitude control
        amp_layout = QHBoxLayout()
        amp_layout.addWidget(QLabel("Amplitude (V):"))
        amp_spin = QDoubleSpinBox()
        amp_spin.setObjectName(f"amp_{channel}")
        amp_spin.setRange(0, 3.3)
        amp_spin.setValue(2)
        amp_spin.setSingleStep(0.1)
        amp_layout.addWidget(amp_spin)
        layout.addLayout(amp_layout)
        
        # Offset control
        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Offset (V):"))
        offset_spin = QDoubleSpinBox()
        offset_spin.setObjectName(f"offset_{channel}")
        offset_spin.setRange(0, 3.3)
        offset_spin.setValue(1.65)
        offset_spin.setSingleStep(0.1)
        offset_layout.addWidget(offset_spin)
        layout.addLayout(offset_layout)
        
        # Apply button
        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(lambda: self.apply_changes(channel))
        layout.addWidget(apply_btn)
        
        group.setLayout(layout)
        parent_layout.addWidget(group)
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
    
    def toggle_connection(self):
        if self.serial_port is None:
            try:
                port = self.port_combo.currentText()
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.connect_btn.setText("Disconnect")
                self.statusBar().showMessage("Connected")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Connection failed: {str(e)}")
        else:
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.setText("Connect")
            self.statusBar().showMessage("Disconnected")
    
    def apply_changes(self, channel):
        if not self.serial_port:
            QMessageBox.warning(self, "Warning", "Please connect to device first")
            return
            
        # Get values from controls
        wave_type = self.findChild(QComboBox, f"wave_{channel}").currentIndex() + 1
        frequency = self.findChild(QDoubleSpinBox, f"freq_{channel}").value()
        amplitude = self.findChild(QDoubleSpinBox, f"amp_{channel}").value()
        offset = self.findChild(QDoubleSpinBox, f"offset_{channel}").value()
        
        # Format command
        command = f"CH{channel}:W{wave_type}:F{frequency:.1f}:A{amplitude:.2f}:O{offset:.2f}\n"
        
        try:
            self.serial_port.write(command.encode())
            self.statusBar().showMessage(f"Settings applied to Channel {channel}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to send command: {str(e)}")
    
    def update_plots(self):
        # Generate example waveform data for visualization
        t = np.linspace(0, 0.01, 1000)
        
        for channel in [1, 2]:
            wave_type = self.findChild(QComboBox, f"wave_{channel}").currentIndex()
            freq = self.findChild(QDoubleSpinBox, f"freq_{channel}").value()
            amp = self.findChild(QDoubleSpinBox, f"amp_{channel}").value()
            offset = self.findChild(QDoubleSpinBox, f"offset_{channel}").value()
            
            # Generate preview waveform
            if wave_type == 0:  # Sine
                y = amp * np.sin(2 * np.pi * freq * t) + offset
            elif wave_type == 1:  # Square
                y = amp * np.sign(np.sin(2 * np.pi * freq * t)) + offset
            elif wave_type == 2:  # Sawtooth
                y = amp * (2 * (freq * t - np.floor(freq * t)) - 1) + offset
            else:  # Pulse
                y = amp * (np.sin(2 * np.pi * freq * t) > 0.9).astype(float) + offset
            
            # Update corresponding plot
            if channel == 1:
                self.plot1.clear()
                self.plot1.plot(t, y, pen='y')
            else:
                self.plot2.clear()
                self.plot2.plot(t, y, pen='c')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaveGeneratorPro()
    window.show()
    sys.exit(app.exec())
