import serial
import time
import threading
import cv2

# Serial port settings
serial_port = 'COM7'  # Adjust the serial port as needed
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
    
    valid_axes = ['X', 'Y', 'Z', 'r', 'T']
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'r', 'T'. ")

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

def open_camera(device_index, width=320, height=240, fps=15):
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        print(f"Unable to open camera at index {device_index}")
    else:
        print(f"Camera opened successfully at index {device_index}")
        # Set resolution and frame rate
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def camera_feed():
    camera_index = 1  # Use the correct camera index for Windows
    camera = open_camera(camera_index)
    if not camera.isOpened():
        print("Failed to open camera.")
        return

    print("Camera opened successfully. Press 'q' to exit.")

    try:
        while True:
            ret, frame = camera.read()
            if not ret or frame is None:
                print(f"Failed to capture image from camera at index {camera_index}.")
                break

            cv2.imshow('Camera Feed', frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

def motor_control(ser):
    while True:
        command = input("Enter 'move' to move the R stage, 'check' to check position, or 'exit' to quit: ").strip().lower()

        if command == 'exit':
            break

        if command == 'move':
            direction = input("Enter direction ('+' for positive or '-' for negative): ").strip()
            displacement = input("Enter displacement (pulse number): ").strip()

            try:
                displacement = int(displacement)
                move_stage(ser, 'r', direction, displacement)
            except ValueError as e:
                print(f"Invalid input: {e}")

        elif command == 'check':
            position = check_position(ser, 'X')
            print(f"Current X position: {position}")

        else:
            print("Invalid command. Please enter 'move', 'check', or 'exit'.")

        time.sleep(0.5)

def main():
    camera_thread = threading.Thread(target=camera_feed)
    camera_thread.start()

    ser = connect_to_controller()
    if ser:
        try:
            motor_control(ser)
        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Failed to connect to the motion controller.")

    camera_thread.join()

if __name__ == "__main__":
    main()
