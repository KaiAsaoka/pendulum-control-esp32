import random
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

from TelemetryDataTransferV1_0 import receive_metadata, start_telemetry, data_buffers, variable_names, sock
from TelemetryConfigV1_0 import TIME_PER_DIV_DEFAULT, NUM_DIVS_DEFAULT

# Predefined color options
COLOR_OPTIONS = {
    "Red": (255, 0, 0),
    "Green": (0, 255, 0),
    "Blue": (0, 0, 255),
    "Yellow": (255, 255, 0),
    "Magenta": (255, 0, 255),
    "Cyan": (0, 255, 255),
    "White": (255, 255, 255),
    "Orange": (255, 165, 0),
}

class TelemetryGUI(QtWidgets.QWidget):
    def __init__(self, variable_names, data_buffers):
        super().__init__()
        self.setWindowTitle("ESP32 Telemetry (Oscilloscope Mode)")
        self.resize(1100, 650)

        self.variable_names = variable_names
        self.data_buffers = data_buffers
        self.var_colors = {}
        self.channel_scales = {}  # per-channel scale factors
        self.channel_color_boxes = {}  # per-channel color selectors

        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        main_split = QtWidgets.QHBoxLayout()
        layout.addLayout(main_split)

        # Left panel: variable checkboxes + scale + color
        self.checkbox_layout = QtWidgets.QVBoxLayout()
        main_split.addLayout(self.checkbox_layout)
        self.checkboxes = {}
        self.selected_vars = []

        for name in self.variable_names:
            row = QtWidgets.QHBoxLayout()
            cb = QtWidgets.QCheckBox(name)
            cb.stateChanged.connect(self.update_selected)
            row.addWidget(cb)
            self.checkboxes[name] = cb

            # Scale input
            scale_input = QtWidgets.QLineEdit("1.0")
            scale_input.setFixedWidth(50)
            scale_input.setPlaceholderText("Scale")
            row.addWidget(scale_input)
            self.channel_scales[name] = scale_input

            # Color selector
            color_box = QtWidgets.QComboBox()
            for color_name in COLOR_OPTIONS.keys():
                color_box.addItem(color_name)
            color_box.setCurrentText("White")
            color_box.currentTextChanged.connect(lambda val, n=name: self.update_channel_color(n, val))
            row.addWidget(color_box)
            self.channel_color_boxes[name] = color_box
            self.var_colors[name] = COLOR_OPTIONS["White"]

            self.checkbox_layout.addLayout(row)

        self.checkbox_layout.addStretch(1)

        # Right panel: plot
        self.plot_widget = pg.PlotWidget(title="Live Telemetry")
        main_split.addWidget(self.plot_widget)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Time (ms)")
        self.curves = {}

        # Controls below the plot
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        # Pause/Resume button
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self.toggle_pause)
        controls.addWidget(self.pause_btn)

        # Timebase selector
        controls.addWidget(QtWidgets.QLabel("Timebase (ms/div):"))
        # self.timebase_box = QtWidgets.QComboBox()
        # self.timebase_values = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000]
        # for v in self.timebase_values:
        #     self.timebase_box.addItem(str(v))
        # self.timebase_box.setCurrentText(str(TIME_PER_DIV_DEFAULT))
        # self.timebase_box.currentTextChanged.connect(self.change_timebase)
        self.timebase_box = QtWidgets.QLineEdit(str(TIME_PER_DIV_DEFAULT))
        controls.addWidget(self.timebase_box)

        self.set_t_range_btn = QtWidgets.QPushButton("Set t-axis")
        self.set_t_range_btn.clicked.connect(self.change_timebase)
        controls.addWidget(self.set_t_range_btn)

        # Y-axis range inputs
        controls.addWidget(QtWidgets.QLabel("Y min:"))
        self.ymin_input = QtWidgets.QLineEdit("-1.0")
        self.ymin_input.setFixedWidth(60)
        controls.addWidget(self.ymin_input)

        controls.addWidget(QtWidgets.QLabel("Y max:"))
        self.ymax_input = QtWidgets.QLineEdit("1.0")
        self.ymax_input.setFixedWidth(60)
        controls.addWidget(self.ymax_input)

        self.set_y_range_btn = QtWidgets.QPushButton("Set Y-axis")
        self.set_y_range_btn.clicked.connect(self.set_y_axis)
        controls.addWidget(self.set_y_range_btn)

        controls.addStretch(1)

        # Initial time window
        # self.time_per_div = TIME_PER_DIV_DEFAULT
        self.time_per_div = float(self.timebase_box.text())
        self.num_divs = NUM_DIVS_DEFAULT
        self.time_window_ms = self.time_per_div * self.num_divs

        # State
        self.paused = False

        # Timer for updating plot
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

        from TelemetryDataTransferV1_0 import send_pulse  # import here to avoid circular issues
        self.send_pulse = send_pulse
        self.esp_addr = None  # will be set from main
        self.pulse_timer = QtCore.QTimer()
        self.pulse_timer.setInterval(1000)  # 1 second
        self.pulse_timer.timeout.connect(self.send_keepalive)
        self.pulse_timer.start()

    # ----------------- GUI methods -----------------
    def toggle_pause(self, checked):
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def change_timebase(self, val):
        # self.time_per_div = int(val)
        self.time_per_div = float(self.timebase_box.text())
        self.time_window_ms = self.time_per_div * self.num_divs

    def set_y_axis(self):
        try:
            ymin = float(self.ymin_input.text())
            ymax = float(self.ymax_input.text())
            self.plot_widget.setYRange(ymin, ymax)
        except ValueError:
            print("Invalid Y-axis values")

    def update_channel_color(self, channel_name, color_name):
        self.var_colors[channel_name] = COLOR_OPTIONS[color_name]
        # Update curve color immediately if already plotted
        if channel_name in self.curves:
            self.curves[channel_name].setPen(pg.mkPen(color=COLOR_OPTIONS[color_name], width=2))

    def update_selected(self):
        self.selected_vars = [name for name, cb in self.checkboxes.items() if cb.isChecked()]

        # Remove unselected curves
        for name in list(self.curves.keys()):
            if name not in self.selected_vars:
                self.plot_widget.removeItem(self.curves[name])
                del self.curves[name]

        # Add curves for new selections
        for name in self.selected_vars:
            if name not in self.curves:
                color = self.var_colors.get(name, (255, 255, 255))
                self.curves[name] = self.plot_widget.plot([], [], pen=pg.mkPen(color=color, width=2), name=name)

    def update_plot(self):
        if self.paused or not self.selected_vars:
            return

        now = None
        for name in self.selected_vars:
            buf = self.data_buffers[name]
            if buf:
                times, values = zip(*buf)
                # Apply per-channel scale
                try:
                    scale = float(self.channel_scales[name].text())
                except ValueError:
                    scale = 1.0
                scaled_values = [v * scale for v in values]

                if now is None:
                    now = times[-1]
                window_start = now - self.time_window_ms
                mask = [t >= window_start for t in times]

                self.curves[name].setData([t for t, m in zip(times, mask) if m],
                                          [v for v, m in zip(scaled_values, mask) if m])

        if now is not None:
            self.plot_widget.setXRange(now - self.time_window_ms, now)
    
    def send_keepalive(self):
        if self.esp_addr:
            try:
                self.send_pulse(self.esp_addr)
            except Exception as e:
                print(f"Failed to send pulse: {e}")


# ----------------- MAIN -----------------
if __name__ == "__main__":
    variable_names, esp_addr = receive_metadata()
    data_buffers = start_telemetry(variable_names, esp_addr)
    print("Started Telemetry!")

    app = QtWidgets.QApplication([])
    gui = TelemetryGUI(variable_names, data_buffers)
    gui.esp_addr = esp_addr   # give GUI the ESP address
    gui.show()
    app.exec()
