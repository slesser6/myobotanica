import serial, time
from .spec import COMMANDS, EXIT_KEY
from .menu import print_menu, execute_command

def menu_loop(ser):
    while True:
        print_menu(COMMANDS, EXIT_KEY)
        choice = input("Choose: ").strip()
        if choice == EXIT_KEY:
            break
        if choice not in COMMANDS:
            print("Invalid.")
            continue
        execute_command(ser, COMMANDS[choice])

def main():
    try:
        with serial.Serial("COM10", 57600, timeout=0.5) as ser:
            print("Serial up. Waiting 2 s for radio…")
            time.sleep(2)
            menu_loop(ser)
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    main()
