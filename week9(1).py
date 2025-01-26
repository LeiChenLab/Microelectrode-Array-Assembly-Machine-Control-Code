import serial
import time
import threading
import cv2

# Serial port settings
controller_port = 'COM9'  # Adjust the serial port as needed
arduino_port = 'COM8'  # Adjust the serial port for Arduino as needed
baud_rate = 9600

relay_state = False
motor_direction = False

# Motor control variables
target_position_counts = 0
target_speed_rpm = 0
gear_ratio = 46.85
encoder_cpr = 48
output_cpr = encoder_cpr * gear_ratio

def send_command(ser, command, wait_response=False):
    try:
        ser.write((command + '\n').encode())
        print(f"Sent command: {command.strip()}")

        if wait_response:
            time.sleep(1)
            response = ser.read(ser.in_waiting).decode()
            cleaned_response = response.replace('\r', '').replace('\n', '').strip()
            print(f"Raw response: '{response}'")
            print(f"Cleaned response: '{cleaned_response}'")
            return cleaned_response
        return ""
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()}: {e}")
        return None

def connect_to_device(port):
    try:
        ser = serial.Serial(port, baud_rate, timeout=2)
        print(f"Trying to open serial port: {port}")
        if ser.is_open:
            print(f"Connected successfully to {port}")
            return ser
        else:
            print(f"Cannot open serial port: {port}")
    except serial.SerialException as e:
        print(f"SerialException for port {port}: {e}")
    return None

def convert_degrees_to_pulses(degrees):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent, 0)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def move_rotary_stage(ser, direction, displacement):
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")
    
    displacement = convert_degrees_to_pulses(displacement)
    axis = 'R'
    command = f"{axis}{direction}{int(displacement)}\r"
    response = send_command(ser, command, wait_response=True)
    if response:
        if "ERR" in response:
            print(f"Error response received: {response}")
        elif response != "OK":
            print(f"Unexpected response: {response}")
    return command

def move_linear_stage(ser, axis, direction, displacement):
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")
    
    valid_axes = ['X', 'Y', 'Z', 'T']
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'T'. ")
    
    command = f"{axis}{direction}{int(displacement)}\r"
    response = send_command(ser, command, wait_response=True)
    if response:
        if "ERR" in response:
            print(f"Error response received: {response}")
        elif response != "OK":
            print(f"Unexpected response: {response}")
    return command

def wait_for_stage_to_finish(ser, axis):
    while True:
        position = check_position(ser, axis)
        if position and not position.startswith("ERR"):
            print(f"Axis {axis} position: {position}")
            time.sleep(0.5)
        else:
            print(f"Axis {axis} movement finished.")
            break

def check_position(ser, axis):
    command = f"?{axis}\r"
    response = send_command(ser, command, wait_response=True)
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

def open_camera(device_index, width=320, height=240, fps=15):
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        print(f"Unable to open camera at index {device_index}")
    else:
        print(f"Camera opened successfully at index {device_index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def camera_feed():
    camera_index = 1
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

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

def motor_control(ser, device_type):
    while True:
        command = input("Enter command ('move_linear','move_rotary', 'stop', 'toggle relay', 'change direction', 'set extruder', or 'exit'): ").strip().lower()

        if command == 'exit':
            break

        if device_type == 'arduino':
            if command == 'toggle relay':
                toggle_relay(ser)
            elif command == 'change direction':
                change_motor_direction(ser)
            elif command == 'set extruder':
                position = input("Enter target position for extruder motor: ").strip()
                extruder_motor(ser, position)
            elif command == 'move':
                target_speed_rpm = int(input("Enter target speed in RPM: ").strip())
                target_position_counts = int(input("Enter target position in revolutions: ").strip()) * output_cpr
                control_motor(ser, target_speed_rpm, target_position_counts)
            elif command == 'stop':
                stop_motor(ser)
            else:
                print("Invalid command for Arduino. Please enter 'move', 'stop', 'toggle relay', 'change direction', 'set extruder', or 'exit'.")

        elif device_type == 'controller':
            if command == 'move_rotary':
                direction = input("Enter direction ('+' for positive or '-' for negative): ").strip()
                displacement = input("Enter displacement (pulse number): ").strip()

                try:
                    displacement = int(displacement)
                    move_rotary_stage(ser, direction, displacement)
                except ValueError as e:
                    print(f"Invalid input: {e}")

        
            elif command == 'move_linear':
                axis = input("Enter axis (X, Y, Z, T): ").strip().upper()
                direction = input("Enter direction ('+' for positive or '-' for negative): ").strip()
                displacement = input("Enter displacement (pulse number): ").strip()

                try:
                    displacement = int(displacement)
                    move_linear_stage(ser, axis, direction, displacement)
                except ValueError as e:
                    print(f"Invalid input: {e}")

            elif command == 'stop':
                stop_motor(ser)

            else:
                print("Invalid command for Controller. Please enter 'move', 'stop', or 'exit'.")

        time.sleep(0.5)

def stop_motor(ser):
    command = "STOP"
    response = send_command(ser, command, wait_response=True)
    if response == "":
        print("No response received for stop command. Motor may have stopped without acknowledgment.")
    else:
        print(f"Stop command response: {response}")
    return response

def toggle_relay(ser):
    global relay_state
    relay_state = not relay_state
    command = "Laser_Relay_On" if relay_state else "Laser_Relay_Off"
    response = send_command(ser, command, wait_response=True)
    return response

def change_motor_direction(ser):
    global motor_direction
    motor_direction = not motor_direction
    direction_command = "F" if motor_direction else "B"
    ser.write((direction_command + '\n').encode())
    direction = "FORWARD" if motor_direction else "BACKWARD"
    print(f"Motor running {direction}")

def extruder_motor(ser, position):
    command = f"Extruder_Motor {position}"
    response = send_command(ser, command, wait_response=True)
    return response

def control_motor(ser, speed_rpm, position_counts):
    try:
        send_command(ser, f"{speed_rpm}", wait_response=True)
        send_command(ser, f"{position_counts / output_cpr}", wait_response=True)
        while True:
            response = send_command(ser, "R", wait_response=True)
            if "Target position reached." in response:
                print("Target position reached.")
                break
            time.sleep(1)
    except ValueError as e:
        print(f"Error in motor control: {e}")

def main():
    camera_thread = threading.Thread(target=camera_feed)
    camera_thread.start()

    arduino_ser = connect_to_device(arduino_port)
    controller_ser = connect_to_device(controller_port)

    if arduino_ser and controller_ser:
        try:
            while True:
                # Move stages to specified positions
                move_linear_stage(controller_ser, 'X', '-', 1000)
                

                move_linear_stage(controller_ser, 'Y', '-', 1000)
                wait_for_stage_to_finish(controller_ser, 'Y')

                move_linear_stage(controller_ser, 'Z', '-', 1000)
                wait_for_stage_to_finish(controller_ser, 'Z')

                move_rotary_stage(controller_ser, '-', 100)
                wait_for_stage_to_finish(controller_ser, 'R')

                move_linear_stage(controller_ser, 'T', '-', 1000)
                wait_for_stage_to_finish(controller_ser, 'T')
                
                # Turn on the laser
                toggle_relay(arduino_ser)
                time.sleep(1)  # Adjust sleep duration as needed
                
                # Turn off the laser
                toggle_relay(arduino_ser)
                time.sleep(1)  # Adjust sleep duration as needed
                
        finally:
            arduino_ser.close()
            controller_ser.close()
            print("Serial connections closed.")
    else:
        print("Failed to connect to one or both devices.")

    camera_thread.join()

if __name__ == "__main__":
    main()
