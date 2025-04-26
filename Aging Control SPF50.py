"""
==========  TODO  ==========



Breakdown or aging?
    Aging options:
    
    - Whether to ramp each device individually
    --> how many times?
    --> stop during ramp if device has "failed"?
    
    - Whether to ramp all of the devices up to a hold voltage together afterwards
    --> Voltage and time?
    --> should bad devices be identified and remove if affecting the other ones?

    Breakdown options:
    - Max voltage
    - Ramp rate
    - threshold current
    + increase multimeter read rate?
    
Self test:
    - Disarm
    - Set power supply to {X} Volts and turn on
    - check that current flowing is close to current limit
    - Arm 
    - check that powersupply voltage is close to the voltage setpoint
    - check that the current flowing is below threshold (close to zero)
    - turn on the discharge resistors
    - check that current flowing is close to the current limit
    - turn off the discharge resistors
    

"""

import datetime
import time
import matplotlib.pyplot as plt
import pyvisa as visa
import serial
import io
import logging
import csv
import threading
import queue
import numpy as np
from colorama import init, Fore, Style

# Initialize colorama (automatically handles Windows/macOS/Linux)
init(autoreset=True)

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class Multimeter:
    def __init__(self, resource):
        """
        Initialize the multimeter using a VISA resource.
        """
        self.resource = resource
        self.configure()

    def configure(self):
        """
        Configure the multimeter with the proper settings.
        """
        try:
            self.resource.write_termination = '\n'
            self.resource.read_termination = '\n'
            self.resource.write("*RST")
            time.sleep(0.5)
            self.resource.write(':CONFigure:CURRent:DC')
            time.sleep(0.1)
            # self.resource.write(':ZERO:AUTO OFF')
            self.resource.write(':SAMPle:COUNt %d' % 1)
            self.resource.write(':CURRent:DC:APER %s' % 'MAX')
            # self.resource.write(':CURRent:DC:APER 0.5')
            self.resource.write(':SAMPle:SOURce %s' % 'IMMediate')
            self.resource.write(':TRIGger:SOURce %s' % 'IMMediate')
            self.resource.write(':TRIG:COUN INF')
            self.resource.write(':FORMat:DATA %s' % 'ASCii')
        except Exception as e:
            logging.error("Error configuring multimeter: %s", e)

    def configure_rapid(self):
        try:
            self.resource.write(':CURRent:DC:APER 0.05')
            self.resource.write(':TRIG:COUN %d' % 1)
        except Exception as e:
            logging.error("Error configuring multimeter: %s", e)

    def initiate(self):
        """
        Initiate a sampling cycle.
        """
        try:
            self.resource.write('INIT')
        except Exception as e:
            logging.error("Error initiating multimeter: %s", e)

    def abort(self):
        """
        Abort the current sampling process.
        """
        try:
            self.resource.write(':ABORt')
        except Exception as e:
            logging.error("Error aborting multimeter: %s", e)

    def get_points(self):
        """
        Query the number of available data points.
        """
        try:
            response = self.resource.query("DATA:POINTS?")
            return int(response.strip())
        except Exception as e:
            logging.error("Error reading data points: %s", e)
            return 0

    def read_value(self, clear_extra=False):
        """
        If data is available, read one measurement. If clear_extra is True,
        any extra points in the buffer will be removed.
        """
        points = self.get_points()
        if points > 0:
            try:
                value = self.resource.query("DATA:REMOVE? 1").strip()
                if clear_extra and points > 1:
                    extra = points - 1
                    if extra > 3:
                        self.resource.query("DATA:REMOVE? " + str(extra))
                return value
            except Exception as e:
                logging.error("Error reading measurement: %s", e)
                return None
        return None


class TemperatureHumiditySensor:
    def __init__(self, port='COM5', baudrate=4800):
        """
        Initialize the temperature/humidity sensor over the specified serial port.
        """
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1)
            self.ser_io = io.TextIOWrapper(
                io.BufferedRWPair(self.ser, self.ser, 1),
                newline='\r',
                line_buffering=True)
            print(f"[TEMP/RH] Connected to: {self.ser.portstr}")
        except Exception as e:
            logging.error(
                "Failed to initialize TemperatureHumiditySensor: %s", e)
            raise

    def read_sensor(self):
        """
        Read a complete sensor data packet.
        Returns a tuple (temperature, humidity) if successful, otherwise None.
        """
        if self.ser.in_waiting:
            try:
                line = self.ser_io.readline()
                if "@" in line:
                    lines = []
                    reading_packet = True
                    while reading_packet:
                        line = self.ser_io.readline()
                        if "$" in line:
                            reading_packet = False
                        elif line != '':
                            lines.append(line.strip('\r').strip())
                    self.ser.reset_input_buffer()
                    if len(lines) >= 4:
                        # Parse temperature and humidity from expected positions.
                        temperature = int(lines[1][3:7], 16) / 100.0
                        humidity = int(lines[3][3:7], 16) / 200.0
                        return temperature, humidity
            except Exception as e:
                logging.error("Error reading temperature/humidity: %s", e)
        return None


class PowerSupply:
    def __init__(self, port="COM6", baudrate=115200, timeout=0.5, charging_curve=None, simulation=False):
        """
        Initialize the power supply via a serial connection.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_retries = 5
        self.retry_dwell = 0.2
        self.clear_to_send = True
        self.last_command = ""
        self.voltage_ramp_rate = 5
        self.current_limit_value = 1.5e-3
        self.voltage_setpoint = 0
        self.segment_start_time = datetime.datetime.now()
        self.prgm_index = 0
        self.sequence_complete = False

        if charging_curve is None:
            # self.charging_curve = [
            #     [0, 0, 0],
            #     [100, 5, 140],
            #     [200, 5, 140],
            #     [250, 2, 240],
            #     [300, 2, 240],
            #     [350, 1, 290],
            #     [400, 1, 350],
            #     [450, 0.75, 360],
            #     [500, 0.5, 400],
            #     [520, 0.1, 600]
            # ]
            # self.charging_curve = [
            #     [0, 0, 0],
            #     [10, 1, 15],
            #     [20, 0.7, 30],
            #     [50, 2, 40],
            #     [150, 5, 40],
            #     [300, 2, 120]
            # ]
            self.charging_curve = [
                [0, 0, 0],
                [100, 5, 140],
                [200, 5, 140],
                [250, 2, 240],
                [300, 2, 240],
                [350, 1, 290],
                [400, 1, 350],
                [450, 0.75, 360],
                [500, 0.5, 400],
                [520, 0.1, 600]
            ]
        else:
            self.charging_curve = charging_curve

        if not simulation:
            try:
                self.serial = serial.Serial(
                    port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            except Exception as e:
                logging.error("Failed to initialize PowerSupply serial: %s", e)
                raise

    def blocking_connect(self):
        """
        Continuously try to connect until the power supply responds.
        """
        try:
            if self.serial.isOpen():
                self.serial.close()
            self.serial.open()
        except Exception as e:
            logging.error("Error opening PowerSupply serial port: %s", e)
            raise

        connected = False
        while not connected:
            try:
                timeout_pings = 5
                self.serial.reset_input_buffer()
                self.send_command("=")
                while self.serial.inWaiting() <= 0 and timeout_pings > 0:
                    timeout_pings -= 1
                    time.sleep(1)
                self.serial.reset_input_buffer()
                time.sleep(1)
                self.serial.write("*IDN?\n".encode())
                scale_response = self.serial_read_line()
                print(scale_response)
                if "FUG" in scale_response:
                    connected = True
                    print("[PS] Connection established!")
                    return True
                else:
                    print("[PS] Not connected!")
                    time.sleep(5)
            except Exception as e:
                logging.error("Error during PowerSupply connection: %s", e)
                time.sleep(5)
        return connected

    def send_command(self, command):
        """
        Send a command to the power supply and perform retries if needed.
        """
        try:
            if not self.clear_to_send:
                # Attempt to clear by checking for the expected response.
                if "E0" in self.serial.read_all().decode(errors='ignore'):
                    self.clear_to_send = True
                else:
                    self.clear_to_send = False
                if not self.clear_to_send:
                    for _ in range(self.max_retries):
                        self.send_no_check(command)
                        time.sleep(self.retry_dwell)
                        response = self.serial.read_all().decode(errors='ignore')
                        if "E0" in response:
                            self.clear_to_send = True
                            break
            self.send_no_check(command)
            if "?" not in command:
                self.clear_to_send = False
            self.last_command = command
        except Exception as e:
            logging.error("Error sending command '%s': %s", command, e)

    def send_no_check(self, command):
        """
        Send a command without checking for a clear-to-send response.
        """
        try:
            self.serial.write((str(command) + "\n*IDN\n").encode())
        except Exception as e:
            logging.error("Error in send_no_check: %s", e)

    def serial_read_line(self):
        """
        Read a line from the power supply serial connection.
        """
        try:
            response = self.serial.readline()
            return response.decode(errors='ignore').strip()
        except Exception as e:
            logging.error("Error reading line from PowerSupply: %s", e)
            return ""

    def on(self):
        self.send_command("F1")

    def off(self):
        self.send_command("F0")

    def voltage_ramp_rate_set(self, rate):
        """
        Set the voltage ramp rate (V/s).
        """
        if rate < 0:
            rate = 0
        self.send_command(">S0R " + str(rate))
        self.voltage_ramp_rate = rate

    def voltage_setpoint_set(self, voltage):
        """
        Set the voltage setpoint (V).
        """
        self.send_command("U" + str(voltage))
        self.voltage_setpoint = voltage

    def current_limit_set(self, current):
        """
        Set the current limit (A).
        """
        self.send_command("I" + str(current))
        self.current_limit_value = current

    def read_current(self):
        """
        Query and return the current reading (A).
        """
        try:
            self.serial.reset_input_buffer()
            self.send_command(">M1?")
            response = self.serial_read_line().replace("M1:", "")
            if "E0" in response:
                response = self.serial_read_line().replace("M1:", "")
            return float(response)
        except Exception as e:
            logging.error("Error reading current: %s", e)
            return None

    def read_voltage(self):
        """
        Query and return the voltage reading (V).
        """
        try:
            self.serial.reset_input_buffer()
            self.send_command(">M0?")
            response = self.serial_read_line().replace("M0:", "")
            if "E0" in response:
                response = self.serial_read_line().replace("M0:", "")
            return float(response)
        except Exception as e:
            logging.error("Error reading voltage: %s", e)
            return None

    def startup(self):
        """
        Perform the power supply startup sequence.
        """
        try:
            self.send_command("S1")
            time.sleep(0.01)
            self.current_limit_set(self.current_limit_value)
            time.sleep(0.01)
            self.voltage_setpoint_set(0)
            time.sleep(0.01)
            self.on()
            time.sleep(0.01)
            self.send_command(">S0B 2")
            time.sleep(0.01)
        except Exception as e:
            logging.error("Error during power supply startup: %s", e)

    def restart_program(self):
        self.sequence_complete = False
        self.prgm_index = 0
        # self.segment_start_time = datetime.datetime.now()
        self.voltage_setpoint_set(0)
        self.run_program()

    def run_program(self):
        """
        Advance the charging curve program based on elapsed time.
        """
        try:
            segment_time = self.charging_curve[self.prgm_index][2]
            if segment_time < 0:
                # a negative segment time means the target voltage should be held indefinetly
                return

            elapsed = (datetime.datetime.now() -
                       self.segment_start_time).total_seconds()

            if elapsed > segment_time:
                self.prgm_index += 1
                if self.prgm_index >= len(self.charging_curve):
                    print("[PS] End of sequence...")
                    self.sequence_complete = True
                    self.prgm_index -= 1
                    return

                print(f"[PS] Next program segment {self.prgm_index}.")
                self.segment_start_time = datetime.datetime.now()
                new_ramp_rate = self.charging_curve[self.prgm_index][1]
                new_voltage = self.charging_curve[self.prgm_index][0]
                self.voltage_ramp_rate_set(new_ramp_rate)
                time.sleep(0.01)
                self.voltage_setpoint_set(new_voltage)
        except Exception as e:
            logging.error("Error in run_program: %s", e)


class Multiplexer:
    def __init__(self, port='COM12', baudrate=115200):
        """
        Initialize the multiplexer serial connection.
        """
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1)
            print(f"[MUX] Connected to: {self.serial.portstr}")
        except Exception as e:
            logging.error("Failed to initialize Multiplexer: %s", e)
            raise

    def arm(self):
        """Arm the multiplexer for operation."""
        self.send_command("ARM")

    def disarm(self):
        """Disarm the multiplexer."""
        self.send_command("DISARM")

    def set_channel(self, channel, state):
        """
        Set a channel to on (1) or off (0).
        channel: 0-7
        state: 0 or 1
        """
        if 0 <= channel <= 7 and state in (0, 1):
            self.send_command(f"WA{channel},{state}")
        else:
            logging.error("Invalid channel or state")

    def discharge(self, state):
        """
        Control the discharge circuit.
        state: 0 (off) or 1 (on)
        """
        if state in (0, 1):
            self.send_command(f"WB2,{state}")
            self.send_command(f"WB1,{state}")
        else:
            logging.error("Invalid state")

    def send_command(self, command):
        """
        Send a command to the multiplexer and verify it was accepted.
        """
        try:
            self.serial.write((command + "\n").encode())
            time.sleep(0.05)  # Short delay for response
            response = self.serial.readline().decode().strip()
            if "OK" not in response:
                logging.error(f"MUX command {command} failed: {response}")
                return False
            return True
        except Exception as e:
            logging.error(f"Error sending MUX command {command}: {e}")
            return False


class DataLogger:
    def __init__(self, test_id, device_names, minimum_measurement_count=60):
        """
        Initialize the logger for multiple devices.
        device_names: list of names for each device position (empty string means unused)
        """
        self.test_id = test_id
        self.device_names = device_names
        self.minimum_measurement_count = minimum_measurement_count
        self.data_files = {}
        self.active_device = None

        # Create a file for each device
        base_filename = datetime.datetime.now().strftime("%y%m%d") + "_" + test_id
        for i, name in enumerate(device_names):
            if name:  # Only create files for named devices
                filename = f"{base_filename}_{name}.csv"
                self.data_files[i] = {
                    'name': name,
                    'filename': filename,
                    'data_points': [],
                    'header_written': False
                }

    def set_active_device(self, device_idx):
        """Set which device is currently being measured."""
        self.active_device = device_idx

    def write_header(self, device_idx):
        """Write header to a device's data file."""
        if device_idx in self.data_files:
            header = f"MeasTime,{self.data_files[device_idx]['name']},Temperature,Humidity,Voltage\n"
            try:
                with open(self.data_files[device_idx]['filename'], "a") as f:
                    f.write(header)
                self.data_files[device_idx]['header_written'] = True
            except Exception as e:
                logging.error(
                    f"Error writing header for device {device_idx}: {e}")

    def log_data(self, line):
        """Log data for the currently active device."""
        if self.active_device is not None and self.active_device in self.data_files:
            if not self.data_files[self.active_device]['header_written']:
                self.write_header(self.active_device)

            self.data_files[self.active_device]['data_points'].append(line)

            if len(self.data_files[self.active_device]['data_points']) >= self.minimum_measurement_count:
                self.push_data_to_file(self.active_device)

    def push_data_to_file(self, device_idx):
        """Write buffered data to file for a specific device."""
        if device_idx in self.data_files and self.data_files[device_idx]['data_points']:
            try:
                with open(self.data_files[device_idx]['filename'], "a") as f:
                    f.writelines(self.data_files[device_idx]['data_points'])
                self.data_files[device_idx]['data_points'] = []
            except Exception as e:
                logging.error(
                    f"Error logging data for device {device_idx}: {e}")


class CurrentPlot:
    def __init__(self, ax, controller, x_scale=None, x_max=None, is_scaled_plot=False):
        self.ax = ax
        self.x_scale = x_scale
        self.is_scaled_plot = is_scaled_plot

        self.color_cycle = iter(plt.cycler('color', plt.cm.tab10.colors))

        if self.is_scaled_plot:
            # Scaled plot properties (single device)
            self.current_device = None
            self.rt_times = []
            self.rt_currents = []
            self.realtime_line, = self.ax.plot(
                [], [], '-', label='Device Name')
            self.ax.legend()
        else:
            # Full plot properties (multiple devices)
            self.device_lines = {}  # {device_name: line_object}

        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Current (uA)')

        if x_max is not None:
            self.ax.set_xlim(0, x_max)

    def update(self, new_reading, time_elapsed):
        device_name, time_val, current_val = new_reading

        if self.is_scaled_plot:
            # Handle scaled plot (single device)
            if device_name != self.current_device:
                # Switch to new device - clear previous data
                self.current_device = device_name
                self.rt_times = []
                self.rt_currents = []
                color = next(self.color_cycle)['color']
                self.realtime_line.set_color(color)
                self.realtime_line.set_label(f'Real-time ({device_name})')

            # Update data
            self.rt_times.append(time_val)
            self.rt_currents.append(current_val*1e6)

            # Trim old data if x_scale is set
            if self.x_scale is not None:
                while self.rt_times and self.rt_times[0] <= time_elapsed - self.x_scale:
                    self.rt_times.pop(0)
                    self.rt_currents.pop(0)

            # Update plot and scaling
            self.realtime_line.set_data(self.rt_times, self.rt_currents)
            if len(self.rt_times) > 1:
                self.ax.set_xlim(min(self.rt_times), min(
                    max(self.rt_times)+60, max(self.rt_times)*2))
                minVal = min(self.rt_currents)
                maxVal = max(self.rt_currents)
                self.ax.set_ylim((minVal-abs(minVal)*0.2),
                                 (maxVal+abs(minVal)*0.3))

            self.realtime_line.set_label(
                f"{device_name}: {self.rt_currents[-1]:.4g}uA")
            # self.realtime_line.set_label(f"hecing")

        else:
            # Handle full plot (multiple devices)
            if device_name not in self.device_lines:
                # Create new line and marker for new device
                color = next(self.color_cycle)['color']
                self.device_lines[device_name], = self.ax.plot(
                    [], [], '-', color=color, label=device_name
                )

            # Get or create device data storage
            line = self.device_lines[device_name]
            xdata, ydata = line.get_data()

            # Update data
            xdata = np.append(xdata, time_val)
            ydata = np.append(ydata, current_val*1e6)
            line.set_data(xdata, ydata)

            # Auto-scale to show all data
            self.ax.relim()
            self.ax.autoscale_view()

        self.ax.legend()


class VoltagePlot:
    def __init__(self, ax, computed_curve, x_scale=None, is_scaled_plot=False, x_max=None):
        self.ax = ax
        self.x_scale = x_scale
        self.is_scaled_plot = is_scaled_plot

        # Common properties
        self.color_cycle = iter(plt.cycler('color', plt.cm.tab10.colors))
        # Convert planned curve data to numpy arrays
        self.planned_curve_time = np.array(computed_curve[0])
        self.planned_curve_voltage = np.array(computed_curve[1])

        # Plot target curve
        self.target_line = self.ax.plot(
            self.planned_curve_time,
            self.planned_curve_voltage,
            'k--',
            label='Target Voltage Curve',
            zorder=1
        )[0]

        if self.is_scaled_plot:
            # Scaled plot properties
            self.current_device = None
            self.rt_times = []
            self.rt_voltages = []
            self.realtime_line, = self.ax.plot(
                [], [],
                '-',
                label='Real-time Voltage',
                zorder=2
            )
            self.cursor_marker, = self.ax.plot(
                [], [],
                'ro',
                label='Current Position',
                zorder=3
            )
        else:
            # Full plot properties
            if x_max is not None:
                self.ax.set_xlim(0, x_max)
            self.device_lines = {}
            self.device_markers = {}

        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Voltage (V)')
        self.ax.legend()

    def update(self, new_reading, time_elapsed):
        device_name, time_val, voltage_val = new_reading

        if self.is_scaled_plot:
            # Handle scaled plot
            if device_name != self.current_device:
                self.current_device = device_name
                self.rt_times = []
                self.rt_voltages = []
                color = next(self.color_cycle)['color']
                self.realtime_line.set_color(color)
                self.realtime_line.set_label(
                    f'Real-time Voltage ({device_name})')
                self.cursor_marker.set_color(color)

            # Update data
            self.rt_times.append(time_val)
            self.rt_voltages.append(voltage_val)
            self.cursor_marker.set_data([time_val], [voltage_val])

            # Trim old data
            if self.x_scale is not None:
                while self.rt_times and self.rt_times[0] <= time_elapsed - self.x_scale:
                    self.rt_times.pop(0)
                    self.rt_voltages.pop(0)

            # Update plot
            self.realtime_line.set_data(self.rt_times, self.rt_voltages)

            if len(self.rt_times) > 1:
                x_min = min(self.rt_times)
                x_max = min(max(self.rt_times)+60, max(self.rt_times)*2)

                # Get visible portion of target curve (now using numpy arrays)
                mask = (self.planned_curve_time >= x_min) & (
                    self.planned_curve_time <= x_max)
                visible_target_times = self.planned_curve_time[mask]
                visible_target_volts = self.planned_curve_voltage[mask]

                # Update view
                self.ax.set_xlim(x_min, x_max)
                if len(visible_target_volts) > 0:
                    y_min = min(min(self.rt_voltages), min(
                        visible_target_volts)) - 25
                    y_max = max(max(self.rt_voltages), max(
                        visible_target_volts)) + 25
                else:
                    y_min = min(self.rt_voltages) - 25
                    y_max = max(self.rt_voltages) + 25
                self.ax.set_ylim(y_min, y_max)
        else:
            # Handle full plot (existing code)
            if device_name not in self.device_lines:
                color = next(self.color_cycle)['color']
                self.device_lines[device_name], = self.ax.plot(
                    [], [], '-', color=color, label=device_name, zorder=2
                )
                self.device_markers[device_name], = self.ax.plot(
                    [], [], 'o', color=color, zorder=3
                )

            line = self.device_lines[device_name]
            marker = self.device_markers[device_name]
            xdata, ydata = line.get_data()

            line.set_data(np.append(xdata, time_val),
                          np.append(ydata, voltage_val))
            marker.set_data([time_val], [voltage_val])

            self.ax.relim()
            self.ax.autoscale_view()

        self.ax.legend()


class DataGUI:
    def __init__(self, controller):
        self.controller = controller
        # Create figure with two subplots.
        self.fig, axs = plt.subplots(2, 2, figsize=(20, 14))
        ax1, ax2, ax3, ax4 = axs.ravel()

        # Plot 2: Charging curve and realtime PS voltage.
        # Precompute the planned charging curve.

        computed_curve = self.compute_planned_curve(
            self.controller.power_supply.charging_curve)

        self.voltage_plots = [VoltagePlot(ax3, computed_curve, x_max=computed_curve[0][-1]),
                              VoltagePlot(ax4, computed_curve, x_scale=300, is_scaled_plot=True)]

        self.current_plots = [CurrentPlot(ax1, self.controller, x_max=computed_curve[0][-1]),
                              CurrentPlot(ax2, self.controller, x_scale=300, is_scaled_plot=True)]

    def compute_planned_curve(self, charging_curve):
        """
        Convert the 2D charging_curve array into a full time/voltage curve.
        Each segment is [final voltage, ramp rate (V/s), total segment time (s)].
        Returns two lists: times and voltages.
        """
        times = [0]
        voltages = [0]
        current_time = 0
        start_voltage = 0
        for seg in charging_curve:
            final_voltage, ramp_rate, seg_total_time = seg
            ramp_duration = (final_voltage - start_voltage) / \
                ramp_rate if ramp_rate != 0 else 0
            plateau_duration = seg_total_time - ramp_duration
            # Ramp segment.
            current_time += ramp_duration
            times.append(current_time)
            voltages.append(final_voltage)
            # Plateau segment.
            if seg_total_time < 0:
                current_time += 300
                times.append(current_time)
                voltages.append(final_voltage)
                return times, voltages

            if plateau_duration > 0:
                current_time += plateau_duration
                times.append(current_time)
                voltages.append(final_voltage)
            start_voltage = final_voltage
        return times, voltages

    def update_plots(self):
        """
        Update both subplots with the latest data.
        """
        try:

            # time_elapsed = (datetime.datetime.now() - self.controller.test_start_time).total_seconds()
            time_elapsed = self.controller.current_reading[0]
            for v_plot in self.voltage_plots:
                v_plot.update(
                    new_reading=self.controller.voltage_reading, time_elapsed=time_elapsed)

            for c_plot in self.current_plots:
                c_plot.update(
                    new_reading=self.controller.current_reading, time_elapsed=time_elapsed)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            logging.error("Error updating : %s", e)


class TestController:
    def __init__(self):
        """
        Initialize all devices and the data logger.
        """
        # Initialize the temperature/humidity sensor
        # self.sensor = TemperatureHumiditySensor(port='COM5', baudrate=4800)

        # Initialize VISA resource manager and multimeters
        self.rm = visa.ResourceManager()
        resources = self.rm.list_resources()
        print(resources)
        if len(resources) < 1:
            raise Exception("Not enough VISA resources found for multimeters.")
        self.current_multimeter = Multimeter(
            self.rm.open_resource(resources[0]))

        # Initialize the power supply
        self.power_supply = PowerSupply(
            port="COM8", baudrate=115200, timeout=0.5)
        self.power_supply.blocking_connect()
        self.power_supply.startup()

        # Initialize the multiplexer
        self.multiplexer = Multiplexer(port='COM14')

        self.run_self_test()

        self.indentify_populated_channels()

        if not self.connected_devices:
            raise Exception("No devices connected to test.")

        # Get device names from user
        self.get_device_names()

        # Filter out empty names (unused positions)
        self.connected_devices = [
            i for i, name in enumerate(self.device_names) if name]
        if not self.connected_devices:
            raise Exception("No devices specified for testing.")

        self.logger = DataLogger(
            test_id=input("Enter test ID:\n>>> "),
            device_names=self.device_names
        )

        # Data lists for GUI plotting:
        # For multimeter current: each tuple is (device_name, elapsed_time, dmm_current)
        self.current_reading = ()
        # For power supply voltage readings: each tuple is (device_name, elapsed_time, ps_voltage)
        self.voltage_reading = ()

        # Record test start time.
        self.test_start_time = datetime.datetime.now()

        self.current_device_idx = None

        self.measurement_interval = 0.05

        self.populated_threshold = 1e-7  # Minimum current for device verification

        charging_curve_duration = 0
        for seg in self.power_supply.charging_curve:
            final_voltage, ramp_rate, seg_total_time = seg
            charging_curve_duration += seg_total_time

        print(
            f"Estimated runtime: roughly {len(self.connected_devices) * charging_curve_duration / 60} minutes")

        self.current_multimeter.configure()
        self.current_multimeter.initiate()

    def run_self_test(self):
        """Run comprehensive self-test of all system components."""

        # Test parameters
        test_voltage = 15.0  # Voltage for testing
        current_tolerance = 0.2  # 20% tolerance for current measurements
        voltage_tolerance = 0.1  # 10% tolerance for voltage measurements
        idle_current_threshold = 50e-6  # Maximum allowed current when system should be idle
        max_retries = 3

        # Header
        print(Fore.CYAN + "\n"*5 + "=" * 50 + "\n"
              + " SELF TEST ".center(50, "~") + "\n"
              + "=" * 50 + Style.RESET_ALL)

        # self.power_supply.current_limit_value

        self.current_multimeter.configure_rapid()
        self.current_multimeter.initiate()

        self.multiplexer.disarm()
        self.power_supply.voltage_ramp_rate_set(100)
        self.power_supply.voltage_setpoint_set(test_voltage)
        self.power_supply.on()

        time.sleep(0.5)

        self.current_multimeter.read_value(clear_extra=True)

        # -------------------------------------------------------------------
        # Test Phase 1: Disarmed state verification
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Disarmed State Test ]" + Style.RESET_ALL)
        print(Fore.LIGHTBLACK_EX + "  - System disarmed, power supply on")

        for attempt in range(max_retries):
            try:
                # Verify current
                self.current_multimeter.initiate()
                time.sleep(0.5)
                current = self.current_multimeter.read_value(clear_extra=True)

                if current is None:
                    raise Exception("No current reading from multimeter")

                current = float(current)
                print(f"  Measured current: {current*1e6:.2f}uA")

                if abs(current - self.power_supply.current_limit_value) > current_tolerance * self.power_supply.current_limit_value:
                    raise Exception(
                        f"Current out of range (expected ~{self.power_supply.current_limit_value*1e3:.1f}mA ± {current_tolerance*100:.0f}%)")

                print(Fore.GREEN + "  ✓ Disarmed current OK" + Style.RESET_ALL)
                break

            except Exception as e:
                if attempt == max_retries - 1:
                    raise Exception(f"Disarmed state test failed: {str(e)}")
                print(Fore.RED + f"  ! Retrying: {str(e)}" + Style.RESET_ALL)
                time.sleep(1)

        # -------------------------------------------------------------------
        # Test Phase 2: Armed state verification
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Armed State Test ]" + Style.RESET_ALL)
        print(Fore.LIGHTBLACK_EX + "  - Arming multiplexer")

        for attempt in range(max_retries):
            try:
                # Arm multiplexer
                self.multiplexer.arm()
                time.sleep(1)  # Allow system to stabilize

                # Verify voltage
                ps_voltage = self.power_supply.read_voltage()
                if ps_voltage is None:
                    raise Exception("No voltage reading from power supply")

                print(f"  Measured voltage: {ps_voltage:.2f}V")

                if abs(ps_voltage - test_voltage) > voltage_tolerance * test_voltage:
                    raise Exception(
                        f"Voltage out of tolerance (expected {test_voltage}V ± {voltage_tolerance*100:.0f}%)")

                # Verify current
                self.current_multimeter.initiate()
                time.sleep(0.5)
                current = self.current_multimeter.read_value(clear_extra=True)

                if current is None:
                    raise Exception("No current reading when armed")

                current = float(current)
                print(f"  Measured current: {current*1e6:.2f}uA")

                if current > idle_current_threshold:
                    raise Exception(
                        f"Current too high (expected <{idle_current_threshold*1e6:.1f}uA)")

                print(
                    Fore.GREEN + "  ✓ Armed state OK (voltage and current)" + Style.RESET_ALL)
                break

            except Exception as e:
                if attempt == max_retries - 1:
                    raise Exception(f"Armed state test failed: {str(e)}")
                print(Fore.RED + f"  ! Retrying: {str(e)}" + Style.RESET_ALL)
                self.multiplexer.disarm()
                time.sleep(0.2)

        # -------------------------------------------------------------------
        # Test Phase 3: Discharge circuit verification
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Discharge Circuit Test ]" + Style.RESET_ALL)
        print(Fore.LIGHTBLACK_EX + "  - Activating discharge resistors")

        for attempt in range(max_retries):
            try:
                # Turn on discharge
                self.multiplexer.discharge(1)
                time.sleep(0.2)  # Allow current to stabilize

                # Verify current
                self.current_multimeter.initiate()
                time.sleep(0.5)
                current = self.current_multimeter.read_value(clear_extra=True)

                if current is None:
                    raise Exception("No current reading with discharge on")

                current = float(current)
                print(f"  Measured current: {current*1e3:.2f}mA")

                if abs(current - self.power_supply.current_limit_value) > current_tolerance * self.power_supply.current_limit_value:
                    raise Exception(
                        f"Current out of range (expected ~{self.power_supply.current_limit_value*1e3:.1f}mA ± {current_tolerance*100:.0f}%)")

                print(Fore.GREEN + "  ✓ Discharge circuit OK" + Style.RESET_ALL)
                break

            except Exception as e:
                if attempt == max_retries - 1:
                    raise Exception(f"Discharge circuit test failed: {str(e)}")
                print(Fore.RED + f"  ! Retrying: {str(e)}" + Style.RESET_ALL)
                time.sleep(1)

        self.multiplexer.discharge(0)

        print(Fore.GREEN + "\n" + "=" * 50 + "\n"
              + " SELF TEST PASSED ".center(50, "~") + "\n"
              + "=" * 50 + Style.RESET_ALL)

    def get_device_names(self):
        """
            Get device names from the user for connected devices in a multiplexer with 8 channels.
            Uses color for better readability.

            self.connected_devices: List of integers representing connected device channels (0-7).

            Updates devices_names:
                List of 8 elements where each connected device has its name in its slot index,
                and None for unconnected channels.
        """
        self.device_names = [None] * 8
        # Header
        print(Fore.CYAN + "\n"*5 + "=" * 50 + "\n"
              + " DEVICE NAMING ".center(50, "~") + "\n"
              + "=" * 50 + Style.RESET_ALL)

        print(Fore.YELLOW + "\n[ Enter Device Names ]" + Style.RESET_ALL)
        print(f"{Fore.LIGHTBLACK_EX}or type 'skip' to ignore\n{Style.RESET_ALL}")
        for idx in self.connected_devices:
            while True:
                try:
                    name = input(
                        f"  {Fore.GREEN}Channel {idx + 1}:{Style.RESET_ALL} "
                    ).strip()

                    if not name:
                        raise ValueError("Name cannot be empty")

                    if name.startswith("$|$"):
                        pass
                        # This should be for pasting in a string containing the names for all 8 channels
                        # return name.removeprefix("$|$").split(" | ")
                    elif name != "skip":
                        self.device_names[idx] = name

                    break
                except ValueError as e:
                    print(f"  {Fore.RED}Error: {e}{Style.RESET_ALL}")

        while True:
            # ---------------- Confirmation Table -----------------
            print(Fore.YELLOW + "\n\n[ Current Names ]" + Style.RESET_ALL)
            print(
                f"\n{Fore.GREEN}  Channel {Style.RESET_ALL}│ {Fore.BLUE}Name{Style.RESET_ALL}")
            print(f"{"─" * 10}┼{"─" * 7}")
            for i in range(8):
                status = (
                    Fore.BLUE + self.device_names[i] + Style.RESET_ALL
                    if self.device_names[i]
                    else Fore.LIGHTBLACK_EX + "----" + Style.RESET_ALL
                )
                print(f" {Fore.GREEN}{i+1:8}{Style.RESET_ALL} " +
                      f"│ {Fore.BLUE}{status}{Style.RESET_ALL}")
            # --------------------------------------------------------

            print(Fore.YELLOW + "\n\n[ Confirm or Modify ]" + Style.RESET_ALL)
            print(
                f"{Fore.LIGHTBLACK_EX}type 'modify' to alter names, [ENTER] to accept\n{Style.RESET_ALL}")

            confirm = input(f">>> ").strip().lower()

            if confirm.startswith("m"):
                print(Fore.YELLOW + "\n[ Modify Channels ]" + Style.RESET_ALL)
                print(
                    f"{Fore.LIGHTBLACK_EX}channel number to modify (1-8) or 'done' to finish\n{Style.RESET_ALL}")
                while True:
                    channel_input = input(f">>> ").strip().lower()

                    if channel_input == "done":
                        break

                    try:
                        channel_to_modify = int(channel_input)
                        if channel_to_modify not in range(1, 9):
                            print(
                                f"  {Fore.RED}Error: Channel must be 1-8{Style.RESET_ALL}")
                            continue

                        # if slot_to_modify not in self.connected_devices:
                        #     print(
                        #         f"  {Fore.RED}Error: No device in this slot{Style.RESET_ALL}")
                        #     continue

                        new_name = input(
                            f"  {Fore.GREEN}New name for channel {channel_to_modify}{Style.RESET_ALL}: "
                        ).strip()

                        if not new_name:
                            print(
                                f"  {Fore.RED}Error: Name cannot be empty{Style.RESET_ALL}")
                            continue
                        elif new_name != "skip":
                            self.device_names[channel_to_modify-1] = new_name

                        print(
                            f"  {Fore.BLUE}Updated channel {channel_to_modify}\n{Style.RESET_ALL}")
                    except ValueError:
                        print(
                            f"  {Fore.RED}Error: Enter a channel number (1-8) or 'done'{Style.RESET_ALL}")
            else:
                break

    def indentify_populated_channels(self):
        """Check whether a device on each channel is present by applying low voltage."""
        # print(Fore.YELLOW + "\n[ Indentify populated channels ]" + Style.RESET_ALL)
        # Header
        print(Fore.CYAN + "\n"*5 + "=" * 50 + "\n"
              + " INDENTIFY POPULATED CHANNELS ".center(50, "~") + "\n"
              + "=" * 50 + Style.RESET_ALL)
        self.power_supply.voltage_ramp_rate_set(100)
        self.power_supply.voltage_setpoint_set(
            15)  # Low voltage for verification
        self.power_supply.on()

        self.current_multimeter.configure_rapid()
        self.current_multimeter.initiate()
        time.sleep(0.5)

        self.current_multimeter.read_value(clear_extra=True)
        self.multiplexer.arm()

        time.sleep(0.5)  # Allow voltage to stabilize

        valid_channels = []

        for i in range(8):
            self.multiplexer.set_channel(i, 1)
            time.sleep(0.2)  # Settling time

            # Measure current
            self.current_multimeter.initiate()
            time.sleep(0.5)
            current = self.current_multimeter.read_value(clear_extra=True)
            if current is None:
                print(f"{Fore.RED}Channel {i + 1}: No reading")
                continue

            try:
                current = float(current)
                if current < self.populated_threshold:
                    print(
                        f"{Fore.RED}Channel {i + 1}: No device connected, low current  || ({(current*1e6):.2e}uA)")
                else:
                    valid_channels.append(i)
                    print(
                        f"{Fore.GREEN}Channel {i + 1}: OK                                || ({(current*1e6):.2e}uA)")
            except ValueError:
                print(f"{Fore.RED}Channel {i + 1}: Invalid current reading")
                self.multiplexer.set_channel(i, 0)

        # Update active devices list to only include verified devices
        self.connected_devices = valid_channels

        # Reset power supply
        self.power_supply.voltage_setpoint_set(0)

        return bool(self.connected_devices)

    def perform_measurement(self):
        """Do one measurement cycle for the current device."""
        if self.current_device_idx is None:
            self.start_next_device_test()
        elif self.current_device_idx == 999:
            return False

        # Update the power supply program
        self.power_supply.run_program()

        if self.power_supply.sequence_complete:
            self.finish_device_test(self.current_device_idx)
            self.start_next_device_test()

        current_value = self.current_multimeter.read_value()
        if current_value is None:
            return False  # Skip if no data

        # Read sensor data
        # sensor_reading = self.sensor.read_sensor()
        # if sensor_reading is not None:
        #     temperature, humidity = sensor_reading
        #     sensor_data_str = f"{temperature},{humidity}"
        # else:
            # sensor_data_str = ","

        sensor_data_str = ","

        ps_voltage = self.power_supply.read_voltage()

        # Log data to file
        log_line = f"{datetime.datetime.now()},{current_value},{sensor_data_str},{ps_voltage}\n"
        self.logger.log_data(log_line)

        # Update readings for GUI
        elapsed = (datetime.datetime.now() -
                   self.test_start_time).total_seconds()
        try:
            self.current_reading = (
                self.device_names[self.current_device_idx], elapsed, float(current_value))
            self.voltage_reading = (
                self.device_names[self.current_device_idx], elapsed, ps_voltage)

            # Print status
            print(f"[{datetime.datetime.now()}] {self.device_names[self.current_device_idx]} | "
                  f"{elapsed:.1f}s | Current: {float(current_value):.4g} | "
                  f"PS Voltage: {ps_voltage:.1f} | Sensor: {sensor_data_str}")
        except Exception as e:
            logging.error("Error converting measurement values: %s", e)

        return True

    def finish_device_test(self, device_idx):
        """Clean up after testing a device."""
        print(
            f"Completing test for device {self.device_names[self.current_device_idx]}")

        # Ramp down voltage
        self.power_supply.voltage_setpoint_set(0)

        # Discharge the system
        self.multiplexer.discharge(1)
        time.sleep(5)
        self.multiplexer.discharge(0)

        # Turn off this device
        self.multiplexer.set_channel(device_idx, 0)

        # Save any remaining data
        if device_idx in self.logger.data_files:
            self.logger.push_data_to_file(device_idx)

    def start_next_device_test(self):
        """Start testing the next device."""
        if self.current_device_idx == None:
            self.current_device_idx = 0
        else:
            self.current_device_idx += 1
            if not self.current_device_idx in self.connected_devices:
                print(f"End of tests")  # ---------------
                self.multiplexer.send_command("WA,255")
                self.power_supply.voltage_ramp_rate_set(1)
                self.power_supply.voltage_setpoint_set(520)
                self.power_supply.current_limit_set(0.5e-3)
                self.current_device_idx = 999
                return

        print(
            f"\nStarting test for device {self.current_device_idx + 1}: {self.device_names[self.current_device_idx]}")

        self.logger.set_active_device(self.current_device_idx)

        # Turn on this device
        self.multiplexer.set_channel(self.current_device_idx, 1)
        time.sleep(5)  # Settling time

        self.test_start_time = datetime.datetime.now()
        self.power_supply.restart_program()
        self.current_multimeter.read_value(clear_extra=True)

    def cleanup(self):
        # Abort multimeter sampling.
        try:
            self.current_multimeter.abort()
        except Exception as e:
            logging.error("Error aborting multimeter sampling: %s", e)
        # Close VISA resources.
        try:
            self.current_multimeter.resource.close()
            self.rm.close()
        except Exception as e:
            logging.error("Error closing VISA resources: %s", e)
        # Close power supply serial connection.
        try:
            if self.power_supply.serial.is_open:
                self.power_supply.off()
                self.power_supply.serial.close()
        except Exception as e:
            logging.error("Error closing power supply serial: %s", e)
        # Close multiplexer serial connection.
        try:
            if self.multiplexer.serial.is_open:
                self.multiplexer.disarm()
                self.multiplexer.serial.close()
        except Exception as e:
            logging.error("Error closing multiplexer serial: %s", e)
        # Close sensor serial connection.
        try:
            if self.sensor.ser.is_open:
                self.sensor.ser.close()
        except Exception as e:
            logging.error("Error closing sensor serial: %s", e)
        # Log the remaining contents of the buffer to the file.
        try:
            self.logger.push_data_to_file(self.current_device_idx)
        except Exception as e:
            logging.error("Error logging buffer to file: %s", e)
        print("Cleanup complete. Program terminated.")


class CSVTestController:
    """
    A test controller that feeds pre-recorded measurement data from a CSV file.
    The CSV file is assumed to have a header like:
      MeasTime, dmm1, dmm2, Temperature, Humidity, Voltage
    and subsequent rows with the corresponding values.
    """

    def __init__(self, csv_filename):
        self.datestamp_format = "%Y-%m-%d %H:%M:%S.%f"
        self.csv_filename = csv_filename
        self.data = []
        self.current_index = 0
        self.load_data()
        print(self.data[0][0])
        self.test_start_time = datetime.datetime.strptime(
            self.data[0][0], self.datestamp_format)
        # Mimic the attributes used for plotting in the original controller.
        self.current_reading = (0, 0, 0)  # (elapsed_time, dmm1, dmm2)
        self.voltage_reading = (0, 0)       # (elapsed_time, ps_voltage)

        self.previous_time = datetime.datetime.now()
        self.measurement_interval = 0.05

        self.power_supply = PowerSupply(simulation=True)

        # Get user inputs for test ID and sample names
        # test_id = input("Enter test ID:\n>>> ")
        # sample1 = input("Sample in position 1:\n>>> ")
        # sample2 = input("Sample in position 2:\n>>> ")

        test_id = "testOOP"
        sample1 = "TPx2"
        sample2 = "TPx1"

        self.logger = DataLogger(test_id, sample1, sample2)

    def load_data(self):
        try:
            with open(self.csv_filename, "r") as f:
                reader = csv.reader(f)
                header = next(reader)  # Skip the header line.
                for row in reader:
                    self.data.append(row)
            if not self.data:
                raise ValueError("CSV file is empty!")
        except Exception as e:
            print("Error loading CSV data:", e)
            raise

    def perform_measurement(self):
        """
        Simulate one measurement cycle using a row from the CSV file.
        The CSV is assumed to be structured as:
        [timestamp, dmm1, dmm2, temperature, humidity, ps_voltage]
        """
        if self.current_index >= len(self.data):
            # Optionally, loop over or stop
            ## TODO---- should end program here instead ---- ##
            return False

        if (datetime.datetime.now() - self.previous_time).total_seconds() < self.measurement_interval:
            return False

        self.previous_time = datetime.datetime.now()

        row = self.data[self.current_index]
        self.current_index += 1

        try:
            # Parse the CSV values.
            # Note: The timestamp from the CSV is not used for elapsed time.
            dmm1_value = float(row[1])
            dmm2_value = float(row[2])
            temperature = row[3]
            humidity = row[4]
            ps_voltage = float(row[5])
        except Exception as e:
            print("Error parsing CSV row:", e)
            return False

        # Update the simulated readings.
        elapsed = (datetime.datetime.strptime(
            row[0], self.datestamp_format) - self.test_start_time).total_seconds()
        self.current_reading = (elapsed, dmm1_value, dmm2_value)
        self.voltage_reading = (elapsed, ps_voltage)

        # Print out a status message (similar to the real measurement cycle).
        # sensor_data_str = f"{temperature},{humidity}"
        # print(f"[{datetime.datetime.now()}] {elapsed:.1f}s | DMM1: {dmm1_value:.4g} | "
        #       f"DMM2: {dmm2_value:.4g} | PS Voltage: {ps_voltage:.1f} | Sensor: {sensor_data_str}")
        return True

    def cleanup(self):
        pass


def main():
    # Initialize the controller and GUI.
    try:
        controller = TestController()
    except serial.SerialException:
        input("[ERROR] One or more serial devices not present. Press enter to run with simulated data:\n>>>")
        controller = CSVTestController(
            csv_filename="250226_TP01,07_Ageing.csv")
    gui = DataGUI(controller)

    # Create a thread-safe queue for measurement data.
    measurement_queue = queue.Queue()

    # Create an Event to signal shutdown.
    stop_event = threading.Event()

    # Measurement thread function.
    def measurement_thread():
        while not stop_event.is_set():
            try:
                # Perform one measurement cycle.
                if controller.perform_measurement():
                    # Package the latest measurement data.
                    measurement_data = {
                        # (device_name, elapsed_time, dmm1)
                        'current': controller.current_reading,
                        # (device_name, elapsed_time, ps_voltage)
                        'voltage': controller.voltage_reading
                    }
                    measurement_queue.put(measurement_data)
            except Exception as e:
                logging.error("Error in measurement thread: %s", e)
            # Sleep briefly to avoid overwhelming the instruments.
            time.sleep(controller.measurement_interval)

    # Start the measurement thread.
    meas_thread = threading.Thread(
        target=measurement_thread, name="MeasurementThread", daemon=True)
    meas_thread.start()

    # GUI update function to be scheduled in the main thread.
    def update_gui():
        latest_data = None
        if plt.fignum_exists(gui.fig.number):
            # Drain the queue to get the most recent data.
            while not measurement_queue.empty():
                latest_data = measurement_queue.get()
                if latest_data is not None:
                    # Extract elapsed time from the current measurement.
                    time_elapsed = latest_data['current'][1]
                    # Update the voltage plots.
                    for v_plot in gui.voltage_plots:
                        v_plot.update(
                            new_reading=latest_data['voltage'], time_elapsed=time_elapsed)
                    # Update the current plots.
                    for c_plot in gui.current_plots:
                        c_plot.update(
                            new_reading=latest_data['current'], time_elapsed=time_elapsed)

            gui.fig.canvas.draw()
            gui.fig.canvas.flush_events()
            # Schedule the next update after 200ms (5 fps).
            gui.fig.canvas.manager.window.after(200, update_gui)
        else:
            logging.info(
                "GUI window closed; continuing measurements in background.")

    # Schedule the first GUI update.
    gui.fig.canvas.manager.window.after(200, update_gui)

    # Start the matplotlib main loop.
    # This call must be in the main thread.

    try:
        plt.show()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted by user (Ctrl+C).")
        stop_event.set()
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()
