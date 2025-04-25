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
