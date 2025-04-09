import serial
import time

def send_command(ser, command, wait_time=2, expect_multi=False, end_keyword=None):
    """
    Sends a command to the RPi and prints all responses until either wait_time expires
    or an end_keyword is detected.
    
    Args:
        ser (serial.Serial): The open serial connection to the RPi.
        command (str): The command string to send (with trailing newline).
        wait_time (float): Maximum seconds to wait if no end_keyword is detected.
        expect_multi (bool): If True, continues reading lines until wait_time expires.
        end_keyword (str): If provided, reading stops when a line exactly matches this keyword.
    
    Returns:
        List[str]: A list of response lines received (excluding the end marker).
    """
    # Flush any stale data before sending
    ser.reset_input_buffer()
    print(f"\nSending command to RPi: {command.strip()}")
    ser.write(command.encode())

    responses = []
    end_time = time.time() + wait_time
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            # Check if we've reached the end marker
            if end_keyword and line == end_keyword:
                break
            print(line)
            responses.append(line)
            # If not expecting multi-line responses, break after one line.
            if not expect_multi:
                break
        # If no data is arriving and we've passed the wait_time, break out.
        if time.time() > end_time:
            break
        time.sleep(0.1)
    if not responses:
        print("No response (timeout).")
    return responses

def run_script_mode(ser):
    """
    Example script mode: sends a sequence of commands.
    Each command on the RPi side should append an end-of-response marker.
    """
    print("\nRunning preconfigured script...")
    # Each tuple: (command, wait_time, end_keyword)
    script_commands = [
        ("FC:ARM\n", 10, "END_RESPONSE"),
        ("FC:TAKEOFF:5\n", 15, "END_RESPONSE"),
        ("FC:MOVE:FWD\n", 5, "END_RESPONSE"),
        ("FC:LAND\n", 10, "END_RESPONSE"),
    ]
    for cmd, wait_sec, end_kw in script_commands:
        send_command(ser, cmd, wait_time=wait_sec, expect_multi=True, end_keyword=end_kw)
        time.sleep(2)
    print("Script mode complete.")

def main():
    port = "COM10"    # Change to "/dev/ttyUSB0" on Linux if needed
    baud_rate = 57600

    try:
        ser = serial.Serial(port, baud_rate, timeout=0.5)
        print(f"Connected to telemetry radio on {port} at {baud_rate} baud.")
    except Exception as e:
        print(f"Error opening serial port {port}: {e}")
        return

    time.sleep(2)
    
    while True:
        print("\n=== Drone Ground Station Menu ===")
        print("1. GETSTATE         - Get current drone status")
        print("2. ARM              - Arm the drone")
        print("3. TAKEOFF          - Take off to specified altitude")
        print("4. MOVE (FWD/BACK)  - Move the drone forward or backward")
        print("5. YAW (LEFT/RIGHT) - Yaw the drone left or right")
        print("6. LAND             - Land the drone")
        print("7. SERVO            - Control water arm servos (pitch1/pitch2)")
        print("8. PUMP             - Turn water pump ON/OFF")
        print("9. Script Mode      - Run a preconfigured sequence")
        print("10. Custom Command  - Send arbitrary command")
        print("11. Exit")
        choice = input("\nEnter choice: ").strip()

        if choice == "1":
            send_command(ser, "FC:GETSTATE\n", wait_time=3, expect_multi=True, end_keyword="END_RESPONSE")
        
        elif choice == "2":
            responses = send_command(ser, "FC:ARM\n", wait_time=10, expect_multi=True, end_keyword="END_RESPONSE")
            # if not responses:
            #     print("Failed to arm within 10 seconds.")
            # elif any("failed" in resp.lower() for resp in responses):
            #     print("Drone reported failure to arm.")
        elif choice == "3":
            # Removing the altitude prompt; the flight controller will use its pre-programmed default.
            send_command(ser, "FC:TAKEOFF\n", wait_time=15, expect_multi=True, end_keyword="END_RESPONSE")

        elif choice == "4":
            # Let the user choose between move forward and move back without needing extra parameters.
            print("4a. MOVE FORWARD")
            print("4b. MOVE BACKWARD")
            sub_choice = input("Enter choice (a or b): ").strip().lower()
            if sub_choice == "a":
                send_command(ser, "FC:MOVEFWD\n", wait_time=5, expect_multi=True, end_keyword="END_RESPONSE")
            elif sub_choice == "b":
                send_command(ser, "FC:MOVEBACK\n", wait_time=5, expect_multi=True, end_keyword="END_RESPONSE")
            else:
                print("Invalid move command choice.")

        elif choice == "5":
            # Let the user choose between yaw left and yaw right.
            print("5a. YAW LEFT")
            print("5b. YAW RIGHT")
            sub_choice = input("Enter choice (a or b): ").strip().lower()
            if sub_choice == "a":
                send_command(ser, "FC:YAWLEFT\n", wait_time=5, expect_multi=True, end_keyword="END_RESPONSE")
            elif sub_choice == "b":
                send_command(ser, "FC:YAWRIGHT\n", wait_time=5, expect_multi=True, end_keyword="END_RESPONSE")
            else:
                print("Invalid yaw command choice.")
        
        elif choice == "6":
            send_command(ser, "FC:LAND\n", wait_time=10, expect_multi=True, end_keyword="END_RESPONSE")
        
        elif choice == "7":
            # Prompt the user for separate angles.
            angle1 = input("Enter servo1 angle (0-180) [PITCH]: ").strip()
            angle2 = input("Enter servo2 angle (0-180) [PITCH2]: ").strip()
            
            # Send the command for servo1.
            response1 = send_command(ser, f"AR:SERVO:PITCH:{angle1}\n", wait_time=3, expect_multi=True, end_keyword="END_RESPONSE")
            print(response1)
            
            # Send the command for servo2.
            response2 = send_command(ser, f"AR:SERVO:PITCH2:{angle2}\n", wait_time=3, expect_multi=True, end_keyword="END_RESPONSE")
            print(response2)
            
        elif choice == "8":
            state = input("Enter 'ON' or 'OFF': ").strip().upper()
            send_command(ser, f"AR:PUMP:{state}\n", wait_time=3, expect_multi=True, end_keyword="END_RESPONSE")
        
        elif choice == "9":
            run_script_mode(ser)
        
        elif choice == "10":
            custom_cmd = input("Enter custom command: ").strip()
            if not custom_cmd.endswith("\n"):
                custom_cmd += "\n"
            send_command(ser, custom_cmd, wait_time=5, expect_multi=True, end_keyword="END_RESPONSE")
        
        elif choice == "11":
            print("Exiting ground station.")
            break
        else:
            print("Invalid choice. Please try again.")

    ser.close()

if __name__ == "__main__":
    main()
