#motion controller loop
import serial
import time
import threading
import cv2

# Serial port settings
controller_port = 'COM7'  # Adjust the serial port as needed
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

def main():
    camera_thread = threading.Thread(target=camera_feed)
    camera_thread.start()

    ser = connect_to_device(controller_port)
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
