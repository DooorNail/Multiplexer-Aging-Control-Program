# Multiplexer Aging Control Program - AI Assistant Guide

## System Overview

This project controls a hardware setup for testing and aging electronic devices (multiplexers). The system consists of:

1. **Power Supply**: Controls voltage to devices under test
2. **Multimeter**: Measures current through devices
3. **Multiplexer**: Routes power to different device channels
4. **Data Logging**: Records measurements to CSV files
5. **Visualization**: Real-time plotting via PyQt5 GUI

## Core Components

### Hardware Interfaces
- `PowerSupply`: Serial interface to control voltage and current limits with charging curve capabilities
- `Multimeter`: VISA interface for current measurements
- `Multiplexer`: Serial interface for channel switching and discharge control

### Data Management
- `DataLogger`: Records test data to CSV files with device-specific files and a group-hold file
- Data format includes timestamp, current, temperature, humidity, and voltage

### Control Logic
- `TestController`: Orchestrates test flow, hardware communication, and data collection
- `StabilityManager`: Monitors device stability during testing
- Testing modes: individual device aging and group hold testing

### Visualization
- `DataGUI` (PyQt5): Shows real-time plots of current and voltage
- `CurrentPlot`/`VoltagePlot`: Handle different plotting modes (scaled, real-time)

## Key Workflows

### Running Tests
1. Launch `Aging Control SPF50.py` which initializes hardware connections
2. Configure test parameters (device names, test type, voltage profiles)
3. Testing process runs through individual device testing then group-hold phases
4. Data is continuously logged to device-specific CSV files

### Updating the Program
The `Script Updater.py` tool fetches the latest version from GitHub and runs it.

## Project Patterns

### Error Handling
- Hardware operations wrapped in try/except blocks
- Serial/VISA commands retry on failure
- Errors logged with details via Python logging module

### Data Collection
- Buffered writes to CSV files (`minimum_measurement_count`)
- Data buffering in memory before batch file writes
- Header writing on first data collection

### Hardware Communication
- Serial communication with retry mechanisms
- Synchronous command/response patterns
- Watchdog heartbeats for safety monitoring

## Code Navigation Tips

- Main execution flow: `Aging Control SPF50.py` -> `TestController`
- Device communication classes: `PowerSupply`, `Multimeter`, `Multiplexer`
- GUI and visualization: `DataGUI`, `CurrentPlot`, `VoltagePlot`
- Device aging logic: Look for `charging_curve` and `run_program` in `PowerSupply`

## Development Guidelines

- Keep hardware operations in try/except blocks for robustness
- Check device connection status before sending commands
- Follow existing logging patterns for operations and errors
- Maintain backward compatibility with CSV data formats
