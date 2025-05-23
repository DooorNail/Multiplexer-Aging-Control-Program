
import datetime
import time
import pyvisa as visa
import serial
import io
import logging
import csv
import threading
import queue
import os
import numpy as np
import pandas as pd
from colorama import init, Fore, Style

# PyQt5 imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QWidget,
                             QHBoxLayout, QLabel, QPushButton, QMessageBox)
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt


from matplotlib.widgets import Button

# Initialize colorama
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

    def configure_current(self):
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
            # self.resource.write(':CURRent:DC:APER %s' % 'MAX')
            self.resource.write(':CURRent:DC:APER 0.2')
            self.resource.write(':SAMPle:SOURce %s' % 'IMMediate')
            self.resource.write(':TRIGger:SOURce %s' % 'IMMediate')
            self.resource.write(':TRIG:COUN INF')
            self.resource.write(':FORMat:DATA %s' % 'ASCii')
        except Exception as e:
            logging.error("Error configuring multimeter: %s", e)
            
    def configure_voltage(self):
        """
        Configure the multimeter with the proper settings.
        """
        try:
            self.resource.write_termination = '\n'
            self.resource.read_termination = '\n'
            self.resource.write("*RST")
            time.sleep(0.5)
            self.resource.write(':CONFigure:VOLTage:DC')
            time.sleep(0.1)
            # self.resource.write(':ZERO:AUTO OFF')
            self.resource.write(':SAMPle:COUNt %d' % 1)
            # self.resource.write(':CURRent:DC:APER %s' % 'MAX')
            self.resource.write(':VOLTage:DC:APER 0.2')
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

            """ 
            CHARGING CURVE has 4 parameters for each stage:
            [0] Voltage Setpoint
            [1] Ramprate (V/s)
            [2] Segment duration, this includes the initial ramp (s)
            [3] Whether dynamic voltage should run during this section (0: no  || 1: yes)
            """

            self.charging_curve = [
                [0, 0, 0, 0],
                [100, 5, 140, 0],
                [200, 2, 140, 0],
                [250, 2, 240, 0],
                [300, 2, 240, 0],
                [350, 1, 290, 0],
                [400, 1, 350, 0],
                [420, 0.5, 300, 0],
                [440, 0.5, 300, 0],
                [460, 0.5, 300, 0],
                [480, 0.5, 300, 0],
                [500, 0.5, 400, 0],
                [520, 0.2, 300, 0],
                [520, 1, 3600, 0]
            ]
##            self.charging_curve = [
##                [0, 0, 0, 0],
##                [520, 2, 300, 0],
##                [520, 0.2, 7200, 1]
##            ]

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
            # print(f"[PS READ] {datetime.datetime.now()}")
            self.serial.reset_input_buffer()
            # print(f"[PS READ] {datetime.datetime.now()}")
            self.send_no_check(">M0?")
            # print(f"[PS READ] {datetime.datetime.now()}")

            for i in range(5):
                response = self.serial_read_line()
                if "M0" in response:
                    return float(response.replace("M0:", ""))

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

    def is_segment_dynamic(self):
        if len(self.charging_curve[self.prgm_index]) >= 4:
            return self.charging_curve[self.prgm_index][3]
        return 0

    def get_segment_voltage_setpoint(self):
        return self.charging_curve[self.prgm_index[0]]

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
                    print(
                        f"{Fore.CYAN}[PS]{Style.RESET_ALL} End of sequence...")
                    self.sequence_complete = True
                    self.prgm_index -= 1
                    return

                print(
                    f"{Fore.CYAN}[PS]{Style.RESET_ALL} Next program segment {self.prgm_index}.")
                self.segment_start_time = datetime.datetime.now()
                new_ramp_rate = self.charging_curve[self.prgm_index][1]
                new_voltage = self.charging_curve[self.prgm_index][0]
                self.voltage_ramp_rate_set(new_ramp_rate)
                time.sleep(0.01)
                self.voltage_setpoint_set(new_voltage)
        except Exception as e:
            logging.error("Error in run_program: %s", e)


class Multiplexer:
    def __init__(self, port='COM7', baudrate=115200):
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

class LivePlotter:
    def __init__(self):
        plt.ion()  # Interactive mode on
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Voltage plot setup
        self.ax1.set_title('Voltage Measurements')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Voltage (V)')
        self.ax1.grid(True)
        self.voltage_line1, = self.ax1.plot([], [], 'b-', label='Multimeter Voltage')
        self.voltage_line2, = self.ax1.plot([], [], 'r-', label='Power Supply Voltage')
        self.ax1.legend()
        
        # Current plot setup
        self.ax2.set_title('Current Measurements')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.grid(True)
        self.current_line1, = self.ax2.plot([], [], 'g-', label='Current 1')
        self.current_line2, = self.ax2.plot([], [], 'm-', label='Current 2')
        self.ax2.legend()
        
        # Data storage
        self.timestamps = []
        self.voltages = []
        self.ps_voltages = []
        self.currents1 = []
        self.currents2 = []
        
        # Manual scaling controls
        self.ax1_scale_button = plt.axes([0.81, 0.55, 0.1, 0.04])
        self.ax2_scale_button = plt.axes([0.81, 0.15, 0.1, 0.04])
        self.btn_scale1 = Button(self.ax1_scale_button, 'Scale V')
        self.btn_scale2 = Button(self.ax2_scale_button, 'Scale I')
        self.btn_scale1.on_clicked(self.scale_voltage_plot)
        self.btn_scale2.on_clicked(self.scale_current_plot)
        
        plt.tight_layout()
    
    def scale_voltage_plot(self, event):
        """Manually scale voltage plot to fit data"""
        if self.voltages:
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.fig.canvas.draw_idle()
    
    def scale_current_plot(self, event):
        """Manually scale current plot to fit data"""
        if self.currents1:
            self.ax2.relim()
            self.ax2.autoscale_view()
            self.fig.canvas.draw_idle()
    
    def update_plots(self, measurements):
        try:
            """Update plots with new data without auto-scaling"""
            # Store data
            elapsed = (measurements['timestamp'] - self.timestamps[0]).total_seconds() if self.timestamps else 0
            self.timestamps.append(elapsed)
            self.voltages.append(measurements['voltage'])
            self.ps_voltages.append(measurements['ps_voltage'])
            self.currents1.append(measurements['current1'])
            self.currents2.append(measurements['current2'])
            
            # Update voltage plot
            self.voltage_line1.set_data(self.timestamps, self.voltages)
            self.voltage_line2.set_data(self.timestamps, self.ps_voltages)
            
            # Update current plot
            self.current_line1.set_data(self.timestamps, self.currents1)
            self.current_line2.set_data(self.timestamps, self.currents2)
            
            # Redraw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            print("eeror during plot")

class MultimeterController:
    def __init__(self):
        self.rm = visa.ResourceManager()
        self.multimeters = {}
        self.power_supply = None
        self.multiplexer = None
        self.plotter = LivePlotter()
        self.stop_requested = False
        
        self.target_dmms = ["voltage","MY60044278","currentdevice"]
        
        
        self.initialize_devices()
        
    def initialize_devices(self):
        """Initialize all connected devices"""
        # Initialize the power supply
        self.power_supply = PowerSupply(
            port="COM6", baudrate=115200, timeout=0.1)
        self.power_supply.blocking_connect()
        self.power_supply.startup()

        # Initialize the multiplexer
        self.multiplexer = Multiplexer(port='COM7')
        
        self.multiplexer.disarm()
        # self.multiplexer.discharge(0)
        
        # self.multiplexer.set_channel(0,1)
        
        resources = self.rm.list_resources()
        for resource in resources:
            if resource in self.target_dmms:  # Typical Keysight multimeter ID
                inst = self.rm.open_resource(resource)
                idn = inst.query("*IDN?")
                if "34465A" in idn:  # Example multimeter model
                    if len(self.multimeters) < 3:  # Only use first 3 found
                        mm = Multimeter(inst)
                        if len(self.multimeters) == 0:
                            mm.configure_voltage()  # First is voltage
                        else:
                            mm.configure_current()  # Others are current
                        mm.initiate()
                        self.multimeters[len(self.multimeters)] = mm
        if len(self.multimeters < 3):
            raise Exception("What the heck, not enough dmms")
                        
        
    def perform_measurement(self):
        try:
            dmmVoltage = self.multimeters[0].read_value()
            # current_value = self.current_multimeter.read_value()
            if dmmVoltage is None:
                return False  # Skip if no data

            measurements = {
                'timestamp': datetime.datetime.now(),
                'voltage': dmmVoltage,
                'current1': self.multimeters[1].read_value(),
                'current2': self.multimeters[2].read_value(),
                'ps_voltage': self.power_supply.read_voltage()
            }

            return measurements
        except Exception as e:
            logging.error("Error reading values: %s", e)

        return False
        
    def ramp_voltage(self, target_voltage, ramp_rate):
        self.power_supply.voltage_ramp_rate_set(1)
        self.power_supply.current_limit_set(50e-3)
        self.power_supply.voltage_setpoint_set(850)
        self.power_supply.on()
        
    def take_measurements(self):
        """Read all multimeters and power supply"""
        measurements = {
            'timestamp': datetime.datetime.now(),
            'voltage': self.multimeters[0].read_value(),
            'current1': self.multimeters[1].read_value(),
            'current2': self.multimeters[2].read_value(),
            'ps_voltage': self.power_supply.read_voltage()
        }
        return measurements
        
    def run_test(self, config):
        """Main test sequence"""
        filename = f"test_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=[
                'timestamp', 'voltage', 'current1', 'current2', 'ps_voltage'
            ])
            writer.writeheader()
            
            # Enable multiplexer channel
            self.multiplexer.set_channel(config['mux_channel'], 1)
            
            # Ramp voltage
            self.ramp_voltage(config['target_voltage'], config['ramp_rate'])
            
            # Measurement loop
            start_time = time.time()
            while time.time() - start_time < config['duration'] and not self.stop_requested:
                measurements = self.take_measurements()
                if measurements:
                    writer.writerow(measurements)
                    self.plotter.update_plots(measurements)
                time.sleep(config['measurement_interval'])
                
            # Clean up
            self.multiplexer.set_channel(config['mux_channel'], 0)
            self.power_supply.voltage_setpoint_set(0)
            self.power_supply.off()
            plt.ioff()  # Turn off interactive mode when done
            plt.show()  # Keep plots open after test completes

if __name__ == "__main__":
    controller = MultimeterController()
    
    test_config = {
        'mux_channel': 0,
        'target_voltage': 500,
        'ramp_rate': 5,
        'duration': 3600,
        'measurement_interval': 0.05  # More frequent updates for smoother plots
    }
    
    try:
        controller.run_test(test_config)
    except KeyboardInterrupt:
        controller.stop_requested = True
        print("Test stopped by user")