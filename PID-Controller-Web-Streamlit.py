import streamlit as st
import pandas as pd
import time
from simple_pid import PID

# --- Streamlit Page Configuration ---
st.set_page_config(page_title="PID Controller Simulator", layout="wide")


# --- Plant Model (The system to be controlled) ---
class PlantModel:
    """
    Represents a first-order physical system model (the "plant").
    It simulates how the system responds to a control signal.
    """
    def __init__(self, time_constant=5.0, gain=1.0, initial_output=0.0):
        self.time_constant = time_constant
        self.gain = gain
        self.current_output = initial_output

    def update(self, input_control, dt):
        """Updates the plant output based on the control signal and time step (dt)."""
        self.current_output += dt * (self.gain * input_control - self.current_output) / self.time_constant
        return self.current_output

# --- Application State Management with st.session_state ---
def initialize_session_state():
    """Sets the initial values for the simulation state if they don't exist."""
    if 'initialized' not in st.session_state:
        st.session_state.initialized = True
        st.session_state.running = False
        st.session_state.kp = 0.5
        st.session_state.ki = 0.5
        st.session_state.kd = 0.1
        st.session_state.setpoint = 50.0
        st.session_state.dt = 0.1
        st.session_state.max_history = 500
        reset_simulation()

def reset_simulation():
    """Resets the simulation data and model instances to their initial conditions."""
    st.session_state.data = pd.DataFrame({
        'Time (s)': [0.0],
        'Setpoint R(t)': [st.session_state.setpoint],
        'Process Variable Y(t)': [0.0],
        'Error E(t)': [st.session_state.setpoint]
    })
    st.session_state.plant = PlantModel(time_constant=5.0, gain=1.0, initial_output=0.0)
    st.session_state.pid = PID(
        Kp=st.session_state.kp, Ki=st.session_state.ki, Kd=st.session_state.kd,
        setpoint=st.session_state.setpoint, sample_time=st.session_state.dt,
        output_limits=(-100, 100)
    )

# Call the initialization function at the start of the script.
initialize_session_state()


# --- User Interface (UI) ---
st.title("🚀 Real-time PID Controller Simulator")
st.markdown("""
This application simulates a **PID (Proportional-Integral-Derivative) controller**. 
Use the controls to tune the PID parameters and observe the system's response.
""")

with st.expander("⚙️ Control Panel", expanded=True):
    col1, col2, col3 = st.columns(3, gap="large")
    with col1:
        st.subheader("Simulation Controls")
        if st.button("▶️ Start", use_container_width=True, disabled=st.session_state.running):
            st.session_state.running = True
            st.rerun()
        if st.button("⏹️ Stop", use_container_width=True, disabled=not st.session_state.running):
            st.session_state.running = False
            st.rerun()
        if st.button("🔄 Reset Simulation", use_container_width=True):
            st.session_state.running = False
            reset_simulation()
            st.rerun()
    with col2:
        st.subheader("PID Parameters")
        st.session_state.kp = st.number_input("Kp (Proportional Gain)", 0.0, 1000.0, st.session_state.kp, 0.1)
        st.session_state.ki = st.number_input("Ki (Integral Gain)", 0.0, 1000.0, st.session_state.ki, 0.1)
        st.session_state.kd = st.number_input("Kd (Derivative Gain)", 0.0, 1000.0, st.session_state.kd, 0.1)
    with col3:
        st.subheader("Setpoint (R(t))")
        st.session_state.setpoint = st.slider("Target Value", 0.0, 100.0, st.session_state.setpoint, 0.5)
        st.info("Fork the repository on [GitHub](https://github.com/anamsigit/realtime-simulator-pid-controller)")

st.markdown("---")

# --- Data Display Area (Metrics and Plot) ---
metrics_placeholder = st.empty()
st.subheader("PID Controller Response")

# **CHANGE 1: Initialize the chart ONCE with the starting data.**
# Prepare the initial data for the chart.
chart_data = st.session_state.data.set_index('Time (s)')
chart_data_to_display = chart_data[['Setpoint R(t)', 'Process Variable Y(t)', 'Error E(t)']]
# Create the chart element that we will update later.
chart_element = st.line_chart(chart_data_to_display)


# --- Main Simulation Logic ---
if st.session_state.running:
    # This loop runs as long as the 'Start' button is active.
    while st.session_state.running:
        # Update PID controller parameters from user inputs.
        st.session_state.pid.tunings = (st.session_state.kp, st.session_state.ki, st.session_state.kd)
        st.session_state.pid.setpoint = st.session_state.setpoint

        # Get the last values for the next step calculation.
        last_row = st.session_state.data.iloc[-1]
        current_pv = last_row['Process Variable Y(t)']
        current_time = last_row['Time (s)'] + st.session_state.dt

        # Calculate PID control output and update the plant model.
        control_output = st.session_state.pid(current_pv)
        new_y = st.session_state.plant.update(control_output, st.session_state.dt)
        error = st.session_state.setpoint - new_y

        # Append new data to the main history DataFrame.
        new_data_row = pd.DataFrame([{'Time (s)': current_time, 'Setpoint R(t)': st.session_state.setpoint,
                                      'Process Variable Y(t)': new_y, 'Error E(t)': error}])
        st.session_state.data = pd.concat([st.session_state.data, new_data_row], ignore_index=True)

        # Limit the data history length.
        if len(st.session_state.data) > st.session_state.max_history:
            st.session_state.data = st.session_state.data.iloc[-st.session_state.max_history:]

        # Update metrics in their placeholder.
        with metrics_placeholder.container():
            m_col1, m_col2, m_col3, m_col4 = st.columns(4)
            m_col1.metric("Setpoint R(t)", f"{st.session_state.setpoint:.1f}")
            m_col2.metric("Process Variable Y(t)", f"{new_y:.2f}", delta=f"{new_y - current_pv:.2f}")
            m_col3.metric("Error E(t)", f"{error:.2f}")
            m_col4.metric("Control Output U(t)", f"{control_output:.2f}")
        
        # **CHANGE 2: Create a DataFrame for the NEW ROW ONLY and add it to the chart.**
        # The index must be the x-axis value (Time).
        new_plot_row = pd.DataFrame(
            {
                'Setpoint R(t)': [st.session_state.setpoint],
                'Process Variable Y(t)': [new_y],
                'Error E(t)': [error]
            },
            index=[current_time]
        )
        chart_element.add_rows(new_plot_row)

        # Pause the script to simulate real-time progression.
        time.sleep(st.session_state.dt)

# --- Static Display When Simulation is Stopped ---
# This block runs when the app loads or after 'Stop'/'Reset' is pressed.
else:
    last_vals = st.session_state.data.iloc[-1]
    with metrics_placeholder.container():
        m_col1, m_col2, m_col3, m_col4 = st.columns(4)
        m_col1.metric("Setpoint R(t)", f"{last_vals['Setpoint R(t)']:.1f}")
        m_col2.metric("Process Variable Y(t)", f"{last_vals['Process Variable Y(t)']:.2f}")
        m_col3.metric("Error E(t)", f"{last_vals['Error E(t)']:.2f}")
        m_col4.metric("Control Output U(t)", "N/A (Stopped)")