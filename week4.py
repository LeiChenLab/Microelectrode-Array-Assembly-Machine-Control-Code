import serial
import time
import subprocess

# Serial port settings
controller_port = 'COM9'  # Adjust the serial port as needed
baud_rate = 9600

def send_command(ser, command):
    try:
        ser.write((command + '\n').encode())
        print(f"Sent command: {command.strip()}")

        # Wait for a response
        time.sleep(1)
        response = ser.read(ser.in_waiting).decode()
        cleaned_response = response.replace('\r', '').replace('\n', '').strip()
        print(f"Raw response: '{response}'")
        print(f"Cleaned response: '{cleaned_response}'")

        return cleaned_response
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()}: {e}")
        return None

def connect_to_device(port):
    try:
        ser = serial.Serial(port, baud_rate)
        print(f"Connected successfully to device on {port}")
        return ser
    except serial.SerialException as e:
        print(f"SerialException for port {port}: {e}")
        return None

def move_stage(ser, axis, direction, displacement):
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")
    valid_axes = ['X', 'Y', 'Z', 'R', 'T']
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'R', 'T'.")
    command = f"{axis}{direction}{displacement}\r"
    ser.write(command.encode())
    print(f"Command sent to move stage: {command.strip()}")

def stop_motor(ser):
    command = "STOP\n"
    response = send_command(ser, command)
    if response == "":
        print("No response received for stop command. Motor may have stopped without acknowledgment.")
    else:
        print(f"Stop command response: {response}")
    return response

def run_camera_script():
    camera_script_path = '/home/palak/Desktop/software/software/camera_feed.py'
    result = subprocess.run(['python3', camera_script_path])
    return result.returncode

def main():
    if run_camera_script() == 0:
        print("Camera script executed successfully. Proceeding to motor control.")
        ser = connect_to_device(controller_port)
        if ser:
            while True:
                command = input("Enter 'move' to move the stage, 'stop' to stop the motor, or 'exit' to quit: ").strip().lower()

                if command == 'exit':
                    break

                if command == 'move':
                    axis = input("Enter axis (X, Y, Z, R, T): ").strip().upper()
                    direction = input("Enter direction ('+' for positive or '-' for negative): ").strip()
                    displacement = input("Enter displacement (pulse number): ").strip()

                    try:
                        displacement = int(displacement)
                        move_stage(ser, axis, direction, displacement)
                    except ValueError as e:
                        print(f"Invalid input: {e}")

                elif command == 'stop':
                    stop_motor(ser)

                else:
                    print("Invalid command. Please enter 'move', 'stop', or 'exit'.")

                time.sleep(0.5)
        else:
            print("Failed to connect to the motion controller.")
    else:
        print("Camera script failed to execute.")

if __name__ == "__main__":
    main()
