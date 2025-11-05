import random
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import csv
from time import perf_counter

from TelemetryDataTransferV1_0 import setup_serial, receive_metadata, receive_pid, send_pid, start_telemetry, data_buffers, variable_names, sock
from TelemetryConfigV1_0 import TIME_PER_DIV_DEFAULT, NUM_DIVS_DEFAULT

# TODO - change checkboxes to turn on/off individual plots rather than data
# TODO - change update_selected function to work with unique plots
# TODO - change update_plot to plot on unique plots

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

## GPT rounding thing

def nice_round(val):
    """
    Rounds a float to a "nice" human-readable format.
    - Small numbers: keep up to 3-5 significant digits
    - Large numbers: no decimal if integer
    """
    if val == 0:
        return "0"
    abs_val = abs(val)
    
    # Determine number of digits to keep based on magnitude
    if abs_val >= 1:
        # Round to 4 significant digits for medium/large numbers
        return str(round(val, 4 - int(len(str(int(abs_val))))))
    elif abs_val < 1:
        # For small numbers, keep 3 significant digits
        # Example: 0.000159999995 -> 0.00016
        from math import log10, floor
        digits = 3
        exponent = floor(log10(abs_val))
        rounded = round(val, -exponent + (digits - 1))
        return str(rounded)
    else:
        return str(val)

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
        self.pid_inputs = {}

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

        # TODO - Add Specific Plot Widgets here 

        # # XY Position
        # self.plot_xy = pg.PlotWidget(title = "XY Position")
        # main_split.addWidget(self.plot_xy)
        # self.plot_xy.setLabel("left", "Y (mm)")
        # self.plot_xy.setLabel("right", "X (mm)")
        # self.xy_vals = {}

        # # Gantry X PID
        # self.plot_gan_x_pid = pg.PlotWidget(title = "Gantry X PID")
        # main_split.addWidget(self.plot_gan_x_pid)
        # self.plot_gan_x_pid.setLabel("left", "Value")
        # self.plot_gan_x_pid.setLabel("right", "Time (ms)")
        # self.gan_x_pid_vals = {}

        # # Gantry Y PID
        # self.plot_gan_y_pid = pg.PlotWidget(title = "Gantry Y PID")
        # main_split.addWidget(self.plot_gan_y_pid)
        # self.plot_gan_y_pid.setLabel("left", "Value")
        # self.plot_gan_y_pid.setLabel("right", "Time (ms)")
        # self.gan_y_pid_vals = {}

        # # Pendulum X PID
        # self.plot_pen_x_pid = pg.PlotWidget(title = "Pendulum X PID")
        # main_split.addWidget(self.plot_pen_x_pid)
        # self.plot_pen_x_pid.setLabel("left", "Value")
        # self.plot_pen_x_pid.setLabel("right", "Time (ms)")
        # self.pen_x_pid_vals = {}

        # # Pendulum Y PID
        # self.plot_pen_y_pid = pg.PlotWidget(title = "Pendulum Y PID")
        # main_split.addWidget(self.plot_pen_y_pid)
        # self.plot_pen_y_pid.setLabel("left", "Value")
        # self.plot_pen_y_pid.setLabel("right", "Time (ms)")
        # self.pen_y_pid_vals = {}

        # # X Angle
        # self.plot_x_angle = pg.PlotWidget(title = "X Angle")
        # main_split.addWidget(self.plot_x_angle)
        # self.plot_x_angle.setLabel("left", "Angle (deg)")
        # self.plot_x_angle.setLabel("right", "Time (ms)")
        # self.x_angle_vals = {}

        # # Y Angle
        # self.plot_y_angle = pg.PlotWidget(title = "Y Angle")
        # main_split.addWidget(self.plot_y_angle)
        # self.plot_y_angle.setLabel("left", "Angle (deg)")
        # self.plot_y_angle.setLabel("right", "Time (ms)")
        # self.y_angle_vals = {}

        # PID Sending
        self.pid_group = QtWidgets.QGroupBox("PID Controls")
        self.pid_group.setCheckable(True)
        self.pid_group.setChecked(False)  # collapsed by default
        self.pid_layout = QtWidgets.QVBoxLayout()
        self.pid_group.setLayout(self.pid_layout)
        main_split.addWidget(self.pid_group)

        pid_axes = ["Set Angle X", "Set Angle Y", "Set PWM X", "Set PWM Y"]
        pid_params = ["P", "I", "D", "LPF", "Windup"]

        for axis in pid_axes:
            axis_group = QtWidgets.QGroupBox(axis + " PID")
            axis_layout = QtWidgets.QHBoxLayout()
            axis_group.setLayout(axis_layout)

            for param in pid_params:
                label = QtWidgets.QLabel(param)
                axis_layout.addWidget(label)
                line_edit = QtWidgets.QLineEdit("0.0")
                line_edit.setFixedWidth(50)
                axis_layout.addWidget(line_edit)
                self.pid_inputs[f"{axis}_{param}"] = line_edit

            self.pid_layout.addWidget(axis_group)

        # Send PID button
        self.send_pid_btn = QtWidgets.QPushButton("Send PID")
        self.send_pid_btn.clicked.connect(self.send_pid_values)
        self.pid_layout.addWidget(self.send_pid_btn)

        # Controls below the plot
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        # Pause/Resume button
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self.toggle_pause)
        controls.addWidget(self.pause_btn)

        self.save_btn = QtWidgets.QPushButton("Save")
        self.save_btn.setCheckable(True)
        self.save_btn.toggled.connect(self.save)
        controls.addWidget(self.save_btn)

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

    def save(self, checked):
        if not checked:
            return

        all_timestamps = sorted(set(t for buf in self.data_buffers.values() for t, _ in buf))
        data_dict = {name: {t: v for t, v in buf} for name, buf in self.data_buffers.items()}
        self.paused = checked
        self.pause_btn.setText("Resume" if checked else "Pause")

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
                header = ["Timestamp (ms)"] + list(self.data_buffers.keys())
                writer.writerow(header)

                for t in all_timestamps:
                    row = [t]
                    for name in self.data_buffers.keys():
                        row.append(data_dict[name].get(t, ""))
                    writer.writerow(row)

            print(f"✅ Telemetry data saved to: {save_path}")
        except Exception as e:
            print(f"❌ Error saving telemetry data: {e}")
        finally:
            self.save_btn.setChecked(False)

    def send_pid_values(self):
        pid_dict = {}
        for key, line_edit in self.pid_inputs.items():
            try:
                pid_dict[key] = float(line_edit.text())
            except ValueError:
                pid_dict[key] = 0.0  # fallback if invalid input

        # Call your send_pid function (make sure it accepts a dictionary)
        try:
            from TelemetryDataTransferV1_0 import send_pid
            send_pid(pid_dict)
            print("✅ PID values sent:", pid_dict)
        except Exception as e:
            print(f"❌ Failed to send PID: {e}")
        

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
                print("\nNext values\n")
                print(perf_counter())
                print("\n")
                print(times, values)
                
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
        else:
            self.send_pulse()


# ----------------- MAIN -----------------
if __name__ == "__main__":
    setup_serial("COM7")
    variable_names, esp_addr = receive_metadata()
    pid_gain_vals = receive_pid()
    print("PID GAINS: ", pid_gain_vals)
    data_buffers = start_telemetry(variable_names, esp_addr)
    print("Started Telemetry!")

    axes = ["Set Angle X", "Set Angle Y", "Set PWM X", "Set PWM Y"]
    params = ["P", "I", "D", "LPF", "Windup"]

    nice_vals = [nice_round(v) for v in pid_gain_vals]

    pid_dict_init = {}
    for i, axis in enumerate(axes):
        for j, param in enumerate(params):
            pid_dict_init[f"{axis}_{param}"] = nice_vals[i*5 + j]

    app = QtWidgets.QApplication([])
    gui = TelemetryGUI(variable_names, data_buffers)
    for key, val in pid_dict_init.items():
        if key in gui.pid_inputs:
            gui.pid_inputs[key].setText(str(val))
    gui.esp_addr = esp_addr   # give GUI the ESP address
    gui.show()
    app.exec()
