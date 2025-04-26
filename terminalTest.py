from colorama import init, Fore, Style

# Initialize colorama (automatically handles Windows/macOS/Linux)
init(autoreset=True)


def get_device_names():
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

    return self.device_names


print(get_self.device_names([0, 1, 7]))


    def run_self_test(self):
        """Run comprehensive self-test of all system components following specified procedure."""
        
        # Header
        print(Fore.CYAN + "\n"*5 + "=" * 50 + "\n"
            + " SELF TEST ".center(50, "~") + "\n"
            + "=" * 50 + Style.RESET_ALL)
        
        # Test parameters
        test_voltage = 15.0  # Voltage for testing
        current_limit = 1.5e-3  # Current limit for testing
        current_tolerance = 0.2  # 20% tolerance for current measurements
        voltage_tolerance = 0.1  # 10% tolerance for voltage measurements
        idle_current_threshold = 100e-9  # 100nA threshold for armed state
        discharge_current_threshold = 0.5e-3  # Expected current when discharge is on
        max_retries = 3
        
        # Configure instruments
        self.current_multimeter.configure_rapid()
        self.current_multimeter.initiate()
        
        # Initialize system state
        print(Fore.YELLOW + "\n[ Initializing Test State ]" + Style.RESET_ALL)
        self.multiplexer.disarm()
        self.power_supply.voltage_ramp_rate_set(100)
        
        self.power_supply.voltage_setpoint_set(test_voltage)
        time.sleep(0.5)

        # -------------------------------------------------------------------
        # Test Phase 1: Disarmed state verification
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Disarmed State Test ]" + Style.RESET_ALL)
        print("  - System disarmed, power supply on")
        
        for attempt in range(max_retries):
            try:
                # Turn on power supply
                self.power_supply.on()
                time.sleep(1)  # Allow current to stabilize
                
                # Verify current
                self.current_multimeter.initiate()
                time.sleep(0.5)
                current = self.current_multimeter.read_value(clear_extra=True)
                
                if current is None:
                    raise Exception("No current reading from multimeter")
                    
                current = float(current)
                print(f"  Measured current: {current*1e3:.2f}mA")
                
                if abs(current - current_limit) > current_tolerance * current_limit:
                    raise Exception(f"Current out of range (expected ~{current_limit*1e3:.1f}mA ± {current_tolerance*100:.0f}%)")
                
                print(Fore.GREEN + "  ✓ Disarmed current OK" + Style.RESET_ALL)
                break
                
            except Exception as e:
                if attempt == max_retries - 1:
                    raise Exception(f"Disarmed state test failed: {str(e)}")
                print(Fore.YELLOW + f"  ! Retrying: {str(e)}" + Style.RESET_ALL)
                time.sleep(1)

        # -------------------------------------------------------------------
        # Test Phase 2: Armed state verification
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Armed State Test ]" + Style.RESET_ALL)
        print("  - Arming multiplexer")
        
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
                    raise Exception(f"Voltage out of tolerance (expected {test_voltage}V ± {voltage_tolerance*100:.0f}%)")
                
                # Verify current
                self.current_multimeter.initiate()
                time.sleep(0.5)
                current = self.current_multimeter.read_value(clear_extra=True)
                
                if current is None:
                    raise Exception("No current reading when armed")
                    
                current = float(current)
                print(f"  Measured current: {current*1e9:.2f}nA")
                
                if current > idle_current_threshold:
                    raise Exception(f"Current too high (expected <{idle_current_threshold*1e9:.1f}nA)")
                
                print(Fore.GREEN + "  ✓ Armed state OK (voltage and current)" + Style.RESET_ALL)
                break
                
            except Exception as e:
                if attempt == max_retries - 1:
                    raise Exception(f"Armed state test failed: {str(e)}")
                print(Fore.YELLOW + f"  ! Retrying: {str(e)}" + Style.RESET_ALL)
                self.multiplexer.disarm()
                time.sleep(1)
                self.multiplexer.arm()
                time.sleep(1)

        # -------------------------------------------------------------------
        # Test Phase 3: Discharge circuit verification
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Discharge Circuit Test ]" + Style.RESET_ALL)
        print("  - Activating discharge resistors")
        
        for attempt in range(max_retries):
            try:
                # Turn on discharge
                self.multiplexer.discharge(1)
                time.sleep(1)  # Allow current to stabilize
                
                # Verify current
                self.current_multimeter.initiate()
                time.sleep(0.5)
                current = self.current_multimeter.read_value(clear_extra=True)
                
                if current is None:
                    raise Exception("No current reading with discharge on")
                    
                current = float(current)
                print(f"  Measured current: {current*1e3:.2f}mA")
                
                if current < discharge_current_threshold:
                    raise Exception(f"Discharge current too low (expected >{discharge_current_threshold*1e3:.1f}mA)")
                
                print(Fore.GREEN + "  ✓ Discharge circuit OK" + Style.RESET_ALL)
                break
                
            except Exception as e:
                if attempt == max_retries - 1:
                    raise Exception(f"Discharge circuit test failed: {str(e)}")
                print(Fore.YELLOW + f"  ! Retrying: {str(e)}" + Style.RESET_ALL)
                time.sleep(1)

        # -------------------------------------------------------------------
        # Clean up and completion
        # -------------------------------------------------------------------
        print(Fore.YELLOW + "\n[ Completing Test ]" + Style.RESET_ALL)
        try:
            print("  - Returning to safe state")
            self.multiplexer.discharge(0)
            self.multiplexer.disarm()
            self.power_supply.voltage_setpoint_set(0)
            time.sleep(0.5)
            
            print(Fore.GREEN + "  ✓ System returned to safe state" + Style.RESET_ALL)
            
            # Final success message
            print(Fore.GREEN + "\n" + "=" * 50 + "\n"
                + " SELF TEST PASSED ".center(50, "~") + "\n"
                + "=" * 50 + Style.RESET_ALL)
            
        except Exception as e:
            raise Exception(f"Failed to return to safe state: {str(e)}")