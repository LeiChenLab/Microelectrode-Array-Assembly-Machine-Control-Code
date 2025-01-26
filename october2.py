#Automatic movement- random steps 

#import matlab.engine
import serial
import time
import threading
import cv2
import random

# Serial port settings
controller_port = 'COM5'  # Adjust the serial port as needed
  # Adjust the serial port for Arduino as needed -com4
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
    axis = 'r'
    command = f"{axis}{direction}{int(displacement)}\r"
    response = send_command(ser, command, wait_response=True)
    if response:
        if "ERR" in response:
            print(f"Error response received: {response}")
        elif response != "OK":
            print(f"Unexpected response: {response}")
    return command

def move_linear_stage(ser, axis, direction, displacement):
    command = f"{axis}{direction}{int(displacement)}\r"
    response = send_command(ser, command, wait_response=True)
    if response:
        if "ERR5" in response:
            print(f"ERR5 encountered for {axis} axis. Trying opposite direction.")
            opposite_direction = '+' if direction == '-' else '-'
            return move_linear_stage(ser, axis, opposite_direction, displacement)
        elif "ERR" in response:
            print(f"Error response received: {response}")
            return False
        elif response != "OK":
            print(f"Unexpected response: {response}")
            return False
    return True


def motor_control(ser):
    axes = ['X', 'Y', 'Z', 'T', 'r']
    directions = ['+', '-']
    displacements = [100, 500, 1000]  # You can adjust these values as needed

    try:
        while True:
            for axis in axes:
                direction = random.choice(directions)
                displacement = random.choice(displacements)
                
                print(f"Attempting to move {axis} axis {direction}{displacement} steps")
                success = move_linear_stage(ser, axis, direction, displacement)
                
                if success:
                    print(f"Successfully moved {axis} axis {direction}{displacement} steps")
                else:
                    print(f"Failed to move {axis} axis {direction}{displacement} steps")
                
                # Wait for the movement to complete
                time.sleep(0.2)  # Adjust this delay as needed
                
            print("Completed one cycle of all axes. Starting again...")
            time.sleep(0.5)  # Pause between full cycles
            
    except KeyboardInterrupt:
        print("Motor control stopped by user.")
    except Exception as e:
        print(f"An error occurred in motor control: {e}")

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
    camera_index = 0
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

def main():
    camera_thread = threading.Thread(target=camera_feed)
    camera_thread.start()
    ser = connect_to_device(controller_port)
    if ser:
        try:
            print("success")
            motor_control(ser)
        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Failed to connect to the selected device.")

    camera_thread.join()
if __name__ == "__main__":
    main()
