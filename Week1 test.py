import serial
import time

# Serial port settings
serial_port = 'COM5'  # Adjust the serial port as needed
baud_rate = 9600

def send_command(ser, command):
    try:
        ser.write(command.encode())
        print(f"Sent command: {command.strip()}")

        # Wait for a response
        time.sleep(2)
        response = ser.read(ser.in_waiting).decode()
        cleaned_response = response.replace('\r', '').replace('\n', '').strip()
        print(f"Raw controller response: '{response}'")
        print(f"Cleaned controller response: '{cleaned_response}'")

        return cleaned_response
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()}: {e}")
        return None

def connect_to_controller():
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Trying to open serial port: {serial_port}")
        if ser.is_open:
            response = send_command(ser, "?R\r")
            if response:
                expected_response = "?ROK"
                if response == expected_response:
                    print(f"Connected successfully to {serial_port}")
                    return ser
                else:
                    print(f"Failed to connect on {serial_port}, response: '{response}' (expected '{expected_response}')")
                    ser.close()
            else:
                print(f"Cannot open serial port: {serial_port}")
    except serial.SerialException as e:
        print(f"SerialException for port {serial_port}: {e}")
    return None

def move_stage(ser, axis, direction, displacement):
    # Validate input direction
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")

    # Validate input axis
    valid_axes = ['X', 'Y', 'Z', 'theta', 'R', 't', 'T1']  # Changed 'T' to 'T1'
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'theta', 'R', 't', 'T1'.")

    # Construct the command string
    command = f"{axis}{direction}{displacement}\r"
    
    # Send the command to move the stage
    ser.write(command.encode())
    print(f"Moving stage on axis: {axis}, direction: {direction}, displacement: {displacement}")

if __name__ == "__main__":
    ser = connect_to_controller()
    if ser:
        # Example movement command, replace 'T1' with the desired axis, direction, and displacement
        move_stage(ser, 'T1', '+', 100)
        ser.close()
    else:
        print("Failed to establish serial connection. Exiting...")
