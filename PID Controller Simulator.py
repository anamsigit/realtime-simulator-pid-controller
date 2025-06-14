import sys
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QSlider, QDoubleSpinBox, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg # Import pyqtgraph
import numpy as np    # Import numpy

from simple_pid import PID

# --- Plant Model (from previous examples, crucial for simulation) ---
class PlantModel:
    def __init__(self, time_constant=5.0, gain=100.0, initial_output=0.0):
        self.time_constant = time_constant
        self.gain = gain
        self.current_output = initial_output

    def update(self, input_control, dt):
        # Simple first-order system: dy/dt = (K*u - y) / tau
        # Approximated as: y_new = y_old + dt * (K*u - y_old) / tau
        self.current_output += dt * (self.gain * input_control - self.current_output) / self.time_constant
        return self.current_output

class PIDControllerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Controller Real-time Simulator") # Title updated
        self.setGeometry(100, 100, 1200, 800) # Increased window size for plot
        self.initUI()
        self.initSimulation() # Renamed from initControl for clarity

    def initUI(self):
        main_layout = QVBoxLayout()

        # Top Control Panel
        control_panel_layout = QHBoxLayout()

        # dt input (read-only for now, but will be used)
        dt_group = QGroupBox("Simulation Parameters")
        dt_layout = QVBoxLayout()
        dt_label = QLabel("dt (s):")
        self.dt_value_display = QLabel("0.1") # Initial value
        dt_layout.addWidget(dt_label)
        dt_layout.addWidget(self.dt_value_display)
        dt_group.setLayout(dt_layout)
        control_panel_layout.addWidget(dt_group)

        # Controller Group Box
        controller_group = QGroupBox("Controller Parameters")
        controller_layout = QVBoxLayout()

        # Kp input
        kp_layout = QHBoxLayout()
        kp_label = QLabel("Kp:")
        self.kp_input = QDoubleSpinBox()
        self.kp_input.setRange(0.0, 1000.0)
        self.kp_input.setSingleStep(0.1)
        self.kp_input.setValue(0.5) # Initial value
        self.kp_input.valueChanged.connect(self.update_pid_params) # Connect to update PID
        kp_layout.addWidget(kp_label)
        kp_layout.addWidget(self.kp_input)
        controller_layout.addLayout(kp_layout)

        # Ki input
        ki_layout = QHBoxLayout()
        ki_label = QLabel("Ki:")
        self.ki_input = QDoubleSpinBox()
        self.ki_input.setRange(0.0, 1000.0)
        self.ki_input.setSingleStep(1.0)
        self.ki_input.setValue(0.5) # Initial value
        self.ki_input.valueChanged.connect(self.update_pid_params) # Connect to update PID
        ki_layout.addWidget(ki_label)
        ki_layout.addWidget(self.ki_input)
        controller_layout.addLayout(ki_layout)

        # Kd input
        kd_layout = QHBoxLayout()
        kd_label = QLabel("Kd:")
        self.kd_input = QDoubleSpinBox()
        self.kd_input.setRange(0.0, 1000.0)
        self.kd_input.setSingleStep(0.1)
        self.kd_input.setValue(0.1) # Initial value
        self.kd_input.valueChanged.connect(self.update_pid_params) # Connect to update PID
        kd_layout.addWidget(kd_label)
        kd_layout.addWidget(self.kd_input)
        controller_layout.addLayout(kd_layout)

        controller_group.setLayout(controller_layout)
        control_panel_layout.addWidget(controller_group)

        # RT Slider (Setpoint)
        rt_group = QGroupBox("Setpoint (R(t))")
        rt_layout = QVBoxLayout()
        rt_label = QLabel("RT Value:")
        self.rt_value_display = QLabel("50.0") # Display actual setpoint
        self.rt_slider = QSlider(Qt.Horizontal)
        self.rt_slider.setRange(0, 1000) # Range 0-100, scaled by /10.0
        self.rt_slider.setValue(500) # Corresponds to 50.0
        self.rt_slider.setTickInterval(100)
        self.rt_slider.setTickPosition(QSlider.TicksBelow)
        self.rt_slider.valueChanged.connect(self.update_rt_value) # Connect to update setpoint
        rt_layout.addWidget(rt_label)
        rt_layout.addWidget(self.rt_value_display)
        rt_layout.addWidget(self.rt_slider)
        rt_group.setLayout(rt_layout)
        control_panel_layout.addWidget(rt_group)

        # Current Values Display (will be dynamic)
        current_values_group = QGroupBox("Current Values")
        current_values_layout = QVBoxLayout()

        self.r_t_display = QLabel("R(t): 0.0") # Will be updated
        self.e_t_display = QLabel("E(t): 0.0") # Will be updated
        self.u_t_display = QLabel("U(t): 0.0") # Will be updated
        self.y_t_display = QLabel("Y(t): 0.0") # Will be updated

        current_values_layout.addWidget(self.r_t_display)
        current_values_layout.addWidget(self.e_t_display)
        current_values_layout.addWidget(self.u_t_display)
        current_values_layout.addWidget(self.y_t_display)
        current_values_group.setLayout(current_values_layout)
        control_panel_layout.addWidget(current_values_group)

        control_panel_layout.addStretch(1) # Push all to left

        main_layout.addLayout(control_panel_layout)

        # Plot Widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.addLegend()
        self.plot_widget.setTitle("PID Controller Response")
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.r_curve = self.plot_widget.plot(pen=pg.mkPen('g', width=2), name='R(t) Setpoint')  # Green for R(t)
        self.y_curve = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name='Y(t) Process Variable')  # Blue for Y(t)
        self.e_curve = self.plot_widget.plot(pen=pg.mkPen('r', width=2), name='E(t) Error')  # Red for E(t)
        main_layout.addWidget(self.plot_widget)
        print("Fork at https://github.com/anamsigit/realtime-simulator-pid-controller")
        print("load...")
        self.setLayout(main_layout)

    def initSimulation(self): # Renamed for clarity (was initControl)
        self.dt = float(self.dt_value_display.text()) # Get dt from display
        
        # Initialize PID controller from simple-pid library
        initial_setpoint = self.rt_slider.value() / 10.0
        self.pid = PID(self.kp_input.value(), self.ki_input.value(), self.kd_input.value(),
                       setpoint=initial_setpoint,
                       sample_time=self.dt)
        
        # Optional: Set output limits for the PID controller
        # Adjust these limits based on what your "plant" can realistically handle
        self.pid.output_limits = (-100, 100) # Example limits for U(t)

        # Initialize Plant Model
        self.plant = PlantModel(time_constant=5.0, gain=1.0, initial_output=0.0) # Adjusted gain for better response with typical setpoints

        # Data for plotting
        self.time_data = np.array([])
        self.r_data = np.array([])
        self.y_data = np.array([])
        self.e_data = np.array([])

        self.max_history_points = 500 # Limit number of points to keep plot responsive

        # Simulation timer
        self.timer = QTimer()
        self.timer.setInterval(int(self.dt * 1000)) # Convert dt (seconds) to milliseconds
        self.timer.timeout.connect(self.mainloop) # Connect to mainloop
        self.timer.start()

        # Set initial setpoint
        self.setpoint = initial_setpoint

    def update_pid_params(self):
        # Update PID controller parameters when spin boxes change
        self.pid.Kp = self.kp_input.value()
        self.pid.Ki = self.ki_input.value()
        self.pid.Kd = self.kd_input.value()
        # simple-pid handles internal state like integral windup

    def update_rt_value(self, value):
        # Update setpoint from slider
        self.setpoint = value / 10.0 # Scale slider value (e.g., 0-1000 -> 0-100)
        self.rt_value_display.setText(f"{self.setpoint:.1f}")
        self.pid.setpoint = self.setpoint # Update setpoint for the PID object

    def mainloop(self): # This method now contains the simulation logic
        current_time = self.time_data[-1] + self.dt if len(self.time_data) > 0 else 0.0

        # Get current process variable (Y(t)) from the plant
        current_pv = self.plant.current_output

        # PID Calculation using simple-pid library
        control_output = self.pid(current_pv)
        
        # Calculate error for display/plotting (simple-pid does this internally, but we calculate it here for display)
        error = self.setpoint - current_pv 

        # Update Plant with the control output
        current_y = self.plant.update(control_output, self.dt)

        # Update displays
        self.r_t_display.setText(f"R(t): {self.setpoint:.1f}")
        self.e_t_display.setText(f"E(t): {error:.2f}")
        self.u_t_display.setText(f"U(t): {control_output:.2f}")
        self.y_t_display.setText(f"Y(t): {current_y:.2f}")

        # Append data for plotting
        self.time_data = np.append(self.time_data, current_time)
        self.r_data = np.append(self.r_data, self.setpoint)
        self.y_data = np.append(self.y_data, current_y)
        self.e_data = np.append(self.e_data, error)

        # Keep only the latest max_history_points data points
        if len(self.time_data) > self.max_history_points:
            self.time_data = self.time_data[-self.max_history_points:]
            self.r_data = self.r_data[-self.max_history_points:]
            self.y_data = self.y_data[-self.max_history_points:]
            self.e_data = self.e_data[-self.max_history_points:]

        # Update plots
        self.r_curve.setData(self.time_data, self.r_data)
        self.y_curve.setData(self.time_data, self.y_data)
        self.e_curve.setData(self.time_data, self.e_data)
    
    # setsetpointx method is removed as it's now handled by update_rt_value connecting to slider

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = PIDControllerGUI()
    gui.show()
    sys.exit(app.exec_())