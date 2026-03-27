import random
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import csv
import numpy as np

#from time import perf_counter

from TelemetryDataTransferV1_0 import setup_serial, receive_metadata, receive_pid, simple_start, send_pid, stop_telemetry, start_telemetry, data_buffers, variable_names, end_telemetry
from TelemetryConfigV1_0 import TIME_PER_DIV_DEFAULT, NUM_DIVS_DEFAULT

from ProcessTelemetry import process

# Predefined color options
COLOR_OPTIONS = {
    "Red": (255, 0, 0),
    "Dark Red": (139, 0, 0),
    "Light Red": (255, 102, 102),
    "Magenta": (255, 0, 255),
    "Green": (0, 255, 0),
    "Dark Green": (0, 128, 0),
    "Light Green": (144, 238, 144),
    "Lime": (128, 255, 0),
    "Blue": (0, 0, 255),
    "Dark Blue": (0, 0, 139),
    "Light Blue": (173, 216, 230),
    "Cyan": (0, 255, 255),
    "Yellow": (255, 255, 0),
    "Dark Yellow": (204, 204, 0),
    "Light Yellow": (255, 255, 153),
    "Orange": (255, 165, 0),
    "White": (255, 255, 255),
    "Black": (0, 0, 0),
    "Teal": (0, 128, 128)
}

VAR_COLORS = {
    "carriageXPosition": "Green",
    "pendulumXAngle": "Red",
    "xAngleError": "Red",
    "xPWMp": "Red",
    "xPWMi": "Dark Red",
    "xPWMd": "Light Red",
    "xPWMout": "Magenta",
    "xPWM": "Magenta",
    "xPositionError": "Green",
    "angleXp": "Green",
    "angleXi": "Dark Green",
    "angleXd": "Light Green",
    "setAngleXOut": "Lime",
    "carriageYPosition": "Blue",
    "pendulumYAngle": "Yellow",
    "yAngleError": "Yellow",
    "yPWMp": "Yellow",
    "yPWMi": "Dark Yellow",
    "yPWMd": "Light Yellow",
    "yPWMout": "Orange",
    "yPWM": "Orange",
    "yPositionError": "Blue",
    "angleYp": "Blue",
    "angleYi": "Dark Blue",
    "angleYd": "Light Blue",
    "setAngleYOut": "Cyan",
}

class XYGraphWindow(QtWidgets.QWidget):
    def __init__(self, data_buffers):
        super().__init__()
        self.setWindowTitle("XY Position View")
        self.resize(600, 600)

        self.data_buffers = data_buffers
        self.trail_length = 200  # number of past points to show in trail

        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        # --- Plot ---
        self.plot_widget = pg.PlotWidget(title="XY Position")
        self.plot_widget.setLabel("left", "Y Position")
        self.plot_widget.setLabel("bottom", "X Position")
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.addLegend()
        layout.addWidget(self.plot_widget)

        # In __init__, after creating self.plot_widget:
        self.plot_widget.disableAutoRange()
        self.plot_widget.setXRange(-275, 275, padding=0)
        self.plot_widget.setYRange(-400, 400, padding=0)

        # Setpoint node - large, bright
        self.setpoint_dot = self.plot_widget.plot(
            [], [],
            pen=None,
            symbol='o',
            symbolSize=18,
            symbolBrush=(255, 255, 0, 255),
            symbolPen=pg.mkPen((200, 200, 0), width=2),
            name="Setpoint"
        )

        # Carriage trail - fading line
        self.trail_curve = self.plot_widget.plot(
            [], [],
            pen=pg.mkPen((0, 180, 255, 120), width=1),
            name="Carriage Trail"
        )

        # Carriage node - smaller, on top of trail
        self.carriage_dot = self.plot_widget.plot(
            [], [],
            pen=None,
            symbol='o',
            symbolSize=10,
            symbolBrush=(0, 180, 255, 255),
            symbolPen=pg.mkPen((0, 120, 200), width=2),
            name="Carriage"
        )

        # --- Controls ---
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self.toggle_pause)
        controls.addWidget(self.pause_btn)

        controls.addWidget(QtWidgets.QLabel("Trail Length:"))
        self.trail_input = QtWidgets.QLineEdit(str(self.trail_length))
        self.trail_input.setFixedWidth(60)
        controls.addWidget(self.trail_input)

        self.set_trail_btn = QtWidgets.QPushButton("Set")
        self.set_trail_btn.clicked.connect(self.update_trail_length)
        controls.addWidget(self.set_trail_btn)

        controls.addStretch(1)

        self.paused = False

        # --- Timer ---
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def toggle_pause(self, checked):
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def update_trail_length(self):
        try:
            self.trail_length = int(self.trail_input.text())
        except ValueError:
            pass

    def update_plot(self):
        if self.paused:
            return

        buf_sx = self.data_buffers.get("SetPositionX")
        buf_sy = self.data_buffers.get("SetPositionY")
        buf_cx = self.data_buffers.get("carriageXPosition")
        buf_cy = self.data_buffers.get("carriageYPosition")

        # --- Setpoint node ---
        if buf_sx and buf_sy:
            sx = buf_sx[-1] / 10
            sy = buf_sy[-1] / 10
            self.setpoint_dot.setData([sx], [sy])

        # --- Carriage trail + node ---
        if buf_cx and buf_cy:
            # Use the shorter of the two buffers to stay in sync
            n = min(len(buf_cx), len(buf_cy), self.trail_length)
            cx_arr = np.array(buf_cx)[-n:] / 10
            cy_arr = np.array(buf_cy)[-n:] / 10

            self.trail_curve.setData(cx_arr, cy_arr)

            # Current position is the last point
            self.carriage_dot.setData([cx_arr[-1]], [cy_arr[-1]])

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

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
        self.pid_initial = {}
        self.pid_inputs = {}

        self.just_resumed = False

        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        main_split = QtWidgets.QHBoxLayout()
        layout.addLayout(main_split)

        # Left panel: variable checkboxes + scale + color
        self.checkbox_layout = QtWidgets.QVBoxLayout()
        main_split.addLayout(self.checkbox_layout)
        self.checkboxes = {}
        self.selected_vars = []

        color_items = list(COLOR_OPTIONS.items())

        for i, name in enumerate(self.variable_names):
            row = QtWidgets.QHBoxLayout()
            cb = QtWidgets.QCheckBox(name)
            cb.setFixedWidth(125)
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
            color_box.setCurrentText(VAR_COLORS.get(name, "White"))
            color_box.setFixedWidth(75)
            color_box.currentTextChanged.connect(lambda val, n=name: self.update_channel_color(n, val))
            row.addWidget(color_box)
            self.channel_color_boxes[name] = color_box
            self.var_colors[name] = COLOR_OPTIONS[VAR_COLORS.get(name, "White")]

            self.checkbox_layout.addLayout(row)

        self.checkbox_layout.addStretch(1)

        # Right panel: plot
        self.plot_widget = pg.PlotWidget(title="Live Telemetry")
        main_split.addWidget(self.plot_widget)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Sample Index")
        self.curves = {}

        # PID Sending
        self.pid_group = QtWidgets.QGroupBox("PID Controls")
        self.pid_layout = QtWidgets.QVBoxLayout()
        self.pid_group.setFixedWidth(275)
        self.pid_group.setLayout(self.pid_layout)
        main_split.addWidget(self.pid_group)

        pid_axes = ["Set Angle X", "Set Angle Y", "Set PWM X", "Set PWM Y"]
        pid_params = ["P", "I", "D", "aP", "aI", "aD", "aO", "Windup"]

        for axis in pid_axes:
            axis_group = QtWidgets.QGroupBox(axis + " PID")
            axis_layout = QtWidgets.QGridLayout()
            axis_group.setLayout(axis_layout)
            
            for i, param in enumerate(pid_params):
                label = QtWidgets.QLabel(param)

                #RC: Shrunk font size/label width to fit all 8 params
                font = label.font()
                font.setPointSize(8)
                label.setFont(font)
                label.setFixedWidth(15)               
                line_edit = QtWidgets.QLineEdit("0.0")
                line_edit.setFixedWidth(40)

                line_edit.textEdited.connect(self.new_pid_val)
                self.pid_inputs[f"{axis}_{param}"] = line_edit

                row = i // 4 # integer division (round down)
                col = (i % 4) * 2  

                axis_layout.addWidget(label, row, col)
                axis_layout.addWidget(line_edit, row, col + 1)

            self.pid_layout.addWidget(axis_group)

        # Send PID button
        self.send_pid_btn = QtWidgets.QPushButton("Send PID")
        self.send_pid_btn.clicked.connect(self.send_pid_values)
        self.pid_layout.addWidget(self.send_pid_btn)
        self.send_pid_btn.setEnabled(False)

        # Controls below the plot
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        # Pause/Resume button
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self.toggle_pause)
        controls.addWidget(self.pause_btn)

        # Start/Stop telem button
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.setCheckable(True)
        self.stop_btn.toggled.connect(self.toggle_stop)
        controls.addWidget(self.stop_btn)

        self.save_btn = QtWidgets.QPushButton("Save")
        self.save_btn.setCheckable(True)
        self.save_btn.toggled.connect(self.save)
        self.save_btn.setEnabled(False)
        controls.addWidget(self.save_btn)

        # Timebase selector
        controls.addWidget(QtWidgets.QLabel("Samples / Window:"))
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
        self.ymin_input = QtWidgets.QLineEdit("-2000.0")
        self.ymin_input.setFixedWidth(60)
        controls.addWidget(self.ymin_input)

        controls.addWidget(QtWidgets.QLabel("Y max:"))
        self.ymax_input = QtWidgets.QLineEdit("2000.0")
        self.ymax_input.setFixedWidth(60)
        controls.addWidget(self.ymax_input)

        self.set_y_range_btn = QtWidgets.QPushButton("Set Y-axis")
        self.set_y_range_btn.clicked.connect(self.set_y_axis)
        controls.addWidget(self.set_y_range_btn)

        self.view_saved_telem_btn = QtWidgets.QPushButton("View Telemetry")
        self.view_saved_telem_btn.setChecked(False)
        self.view_saved_telem_btn.clicked.connect(self.view_telem)
        self.view_saved_telem_btn.setEnabled(False)
        controls.addWidget(self.view_saved_telem_btn)

        self.view_xy_btn = QtWidgets.QPushButton("View XY")
        self.view_xy_btn.clicked.connect(self.open_xy_view)
        controls.addWidget(self.view_xy_btn)

        controls.addStretch(1)

        # Initial time window
        # self.time_per_div = TIME_PER_DIV_DEFAULT
        self.time_per_div = float(self.timebase_box.text())
        self.num_divs = NUM_DIVS_DEFAULT
        self.time_window_ms = self.time_per_div

        # State
        self.paused = False

        # Timer for updating plot
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    # ----------------- GUI methods -----------------
    def toggle_pause(self, checked):
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

    def reset_buffer(self):
        for name in self.variable_names:
            self.data_buffers[name].clear()

    def toggle_stop(self, checked):
        self.stop_btn.setText("Start" if checked else "Stop")
        self.paused = checked
        if(checked):
            stop_telemetry()
            self.send_pid_btn.setEnabled(True)
            self.save_btn.setEnabled(True)
            self.view_saved_telem_btn.setEnabled(True)
        else:
            self.just_resumed = True
            simple_start()
            self.send_pid_btn.setEnabled(False)
            self.save_btn.setEnabled(False)
            self.view_saved_telem_btn.setEnabled(False)

    def new_pid_val(self):
        line_edit = self.sender()
        line_edit.setStyleSheet("background-color: red;")

    def view_telem(self):
        process()

    def save(self, checked):
        if not checked:
            return
        
        buffers_snapshot = {name: list(buf) for name, buf in self.data_buffers.items()}

        self.paused = True
        self.pause_btn.setText("Resume")

        file_dialog = QtWidgets.QFileDialog(self)
        file_dialog.setAcceptMode(QtWidgets.QFileDialog.AcceptMode.AcceptSave)
        file_dialog.setNameFilter("CSV files (*.csv)")
        file_dialog.setDefaultSuffix("csv")
        file_dialog.setWindowTitle("Save Telemetry Data")

        if file_dialog.exec():
            save_path = file_dialog.selectedFiles()[0]
        else:
            self.save_btn.setChecked(False)
            return

        try:
            with open(save_path, "w", newline="") as f:
                writer = csv.writer(f)

                # --- Write telemetry data ---
                names = list(buffers_snapshot.keys())
                header = ["Sample Index"] + names
                writer.writerow(header)

                # All buffers should be the same length, use the longest as reference
                max_len = max(len(buf) for buf in buffers_snapshot.values())
                buffers_as_lists = {name: list(buffers_snapshot[name]) for name in names}

                for i in range(max_len):
                    row = [i]
                    for name in names:
                        buf = buffers_as_lists[name]
                        row.append(buf[i] if i < len(buf) else "")
                    writer.writerow(row)

                # --- Add a blank line separator ---
                writer.writerow([])

                # --- Write PID values ---
                axes = ["Set Angle X", "Set Angle Y", "Set PWM X", "Set PWM Y"]
                params = ["P", "I", "D", "aP", "aI", "aD", "aO", "Windup"]

                writer.writerow(["PID Parameters"])
                writer.writerow(["Axis"] + params)

                pid_dict = {}
                for key, line_edit in self.pid_inputs.items():
                    try:
                        pid_dict[key] = float(line_edit.text())
                    except ValueError:
                        pid_dict[key] = self.pid_initial.get(key, 0.0)

                for axis in axes:
                    row = [axis] + [pid_dict.get(f"{axis}_{param}", 0.0) for param in params]
                    writer.writerow(row)

            self.reset_buffer()
            print(f"Telemetry data + PID values saved to: {save_path}")

        except Exception as e:
            print(f"Error saving telemetry data: {e}")

        finally:
            self.save_btn.setChecked(False)

    def send_pid_values(self):
        pid_dict = {}
        for key, line_edit in self.pid_inputs.items():
            try:
                pid_dict[key] = float(line_edit.text())
            except ValueError:
                pid_dict[key] = self.pid_initial[key]

        for line_edit in self.pid_inputs.values():
            line_edit.setStyleSheet("")
            
        send_pid(pid_dict)

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

        total_samples = 0

        for name in self.selected_vars:
            buf = self.data_buffers[name]
            if not buf:
                continue

            data = np.array(buf)
            total_samples = max(total_samples, len(data))
            display_count = int(self.time_window_ms)
            data = data[-display_count:]

            try:
                scale = float(self.channel_scales[name].text())
            except ValueError:
                scale = 1.0

            scaled_values = data * scale

            # Anchor x indices to absolute position in buffer
            start_idx = total_samples - len(scaled_values)
            x_indices = np.arange(start_idx, total_samples)
            self.curves[name].setData(x_indices, scaled_values)

        if total_samples > 0:
            display_count = int(self.time_window_ms)
            # Window scrolls once buffer exceeds display_count
            x_max = max(total_samples, display_count)
            x_min = x_max - display_count
            self.plot_widget.setXRange(x_min, x_max, padding=0)

    def open_xy_view(self):
        if not hasattr(self, '_xy_window') or not self._xy_window.isVisible():
            self._xy_window = XYGraphWindow(self.data_buffers)
            self._xy_window.show()
        else:
            self._xy_window.raise_()
            self._xy_window.activateWindow()
# ----------------- MAIN -----------------
if __name__ == "__main__":
    setup_serial("COM3")
    variable_names, esp_addr = receive_metadata()
    pid_gain_vals = receive_pid()
    print("PID GAINS: ", pid_gain_vals)
    data_buffers = start_telemetry(variable_names, esp_addr)
    print("Started Telemetry!")

    axes = ["Set Angle X", "Set Angle Y", "Set PWM X", "Set PWM Y"]
    params = ["P", "I", "D", "aP", "aI", "aD", "aO", "Windup"]

    ## This part probably don't need but I can't test right now 
    pid_dict_init = {}
    for i, axis in enumerate(axes):
        for j, param in enumerate(params):
            pid_dict_init[f"{axis}_{param}"] = pid_gain_vals[i*8 + j]
    
    app = QtWidgets.QApplication([])
    gui = TelemetryGUI(variable_names, data_buffers)
    gui.pid_initial = pid_dict_init
    for key, val in pid_dict_init.items():
        if key in gui.pid_initial:
            gui.pid_inputs[key].setText(str(val))

    gui.esp_addr = esp_addr   # give GUI the ESP address
    gui.show()
    app.exec()