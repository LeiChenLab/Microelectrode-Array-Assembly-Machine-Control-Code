import serial
import time

# Serial port settings
serial_port = 'COM9' # Adjust the serial port as needed
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
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")
    
    valid_axes = ['X', 'Y', 'Z', 'R', 'T']
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'R', 'T'. ")

    if axis == 'R':
        displacement = convert_degrees_to_pulses(displacement)

    command = f"{axis}{direction}{int(displacement)}\r"
    response = send_command(ser, command)
    if response:
        if "ERR" in response:
            handle_error(ser, response)
        elif response != "OK":
            print(f"Unexpected response: {response}")
    return command

def convert_degrees_to_pulses(degrees):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent, 0)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def check_position(ser, axis):
    command = f"?{axis}\r"
    response = send_command(ser, command)
    if response and response.startswith(f"?{axis}"):
        position = response.split()[-1]
        return position
    elif response == "OK":
        print(f"Move command acknowledged but position not returned for axis {axis}.")
        return "Position not returned, OK acknowledged"
    elif "ERR" in response:
        print(f"Error response received: {response}")
        return response
    else:
        print(f"Failed to get position for axis {axis}.")
        return None

def handle_error(ser, response):
    print(f"Handling error: {response}")
    if "ERR1" in response:
        print("Received ERR1, attempting to re-home the R axis.")
        home_axis(ser, 'R')
        time.sleep(5)
    else:
        print(f"Unhandled error response: {response}")

def home_axis(ser, axis):
    print(f"Attempting to home {axis} axis.")
    command = f"H{axis}\r"
    response = send_command(ser, command)
    if response == "OK":
        print(f"{axis} axis homed successfully.")
    else:
        print(f"Failed to home {axis} axis. Response: {response}")

if __name__ == "__main__":
    ser = connect_to_controller()
    if ser:
        print("Moving T stage...")
        move_stage(ser, 'T', '-', 1200)
        
        position = check_position(ser, 'X')
        if position:
            print(f"Current position of T stage: {position}")
        else:
            print("Failed to get position for axis X after multiple attempts.")

        time.sleep(5)

        position = check_position(ser, 'R')
        if position:
            if "ERR" in position:
                handle_error(ser, position)
            else:
                print(f"Current position of R (theta) stage: {position}")
        else:
            print("Failed to get position for axis R after multiple attempts.")

        ser.close()
    else:
        print("Failed to establish serial connection. Exiting...")
