import serial
import time

# Serial port settings
#serial_port = '/dev/ttyUSB0'  # Adjust the serial port as needed
serial_port = 'COM9'
baud_rate = 9600

def send_command(ser, command):
    try:
        ser.write((command + '\r').encode())
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
            response = send_command(ser, "?R")
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
'''
def query_error(ser):
    command = "ERROR"
    response = send_command(ser, command)
    if response:
        print(f"Error status: {response}")

def initialize_controller(ser):
    command = "INIT"
    response = send_command(ser, command)
    if response == "OK":
        print("Controller initialized successfully.")
    else:
        print(f"Failed to initialize controller. Response: {response}")
'''
def home_axis(ser, axis):
    print(f"Attempting to home {axis} axis.")
    command = f"H{axis}"
    response = send_command(ser, command)
    if response == "OK":
        print(f"{axis} axis homed successfully.")
    else:
        print(f"Failed to home {axis} axis. Response: {response}")

def move_R_axis(ser, direction, degrees):
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")

    # Rotary stage specifications
    stepper_angle_deg = 1.8  # From the structure
    transmission_ratio = 180  # From the structure
    subdivision = 8  # From the structure
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent, 0)
    print(f"Converting {degrees} degrees to {pulses} pulses.")

    command = f"R{direction}{int(pulses)}"
    response = send_command(ser, command)
    if response:
        if "ERR1" in response:
            print("Received ERR1, attempting to re-home the R axis.")
            home_axis(ser, 'r')
            time.sleep(5)  # Wait for homing to complete
            # Retry the move after homing
            response = send_command(ser, command)
            if response and "ERR" in response:
                print(f"Error response received after re-homing: {response}")
        elif "ERR" in response:
            print(f"Error response received: {response}")
        elif response != "OK":
            print(f"Unexpected response: {response}")

if __name__ == "__main__":
    ser = connect_to_controller()
    if ser:
        # Query error status
        #query_error(ser)

        # Initialize controller
        #initialize_controller(ser)

        # Try homing the R axis
        #home_axis(ser, 'R')

        # Try to move the R axis using pulse equivalent
        move_R_axis(ser, '-', 90)  # Move R axis 90 degrees in the positive direction

        ser.close()
    else:
        print("Failed to establish serial connection. Exiting...")
