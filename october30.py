import serial
import threading
import tkinter as tk
from tkinter import messagebox
import cv2
import numpy as np
import time

# Global serial variable
ser = None
serial_lock = threading.Lock()
camera_running = False

# Command cooldown time
last_command_time = 0
command_cooldown = 0.5  # seconds

# HSV calibration values
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([179, 255, 255])

def connect_to_device(port):
    global ser
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        print(f"Connected successfully to {port}")
        return True
    except serial.SerialException as e:
        print(f"Connection error: {e}")
        messagebox.showerror("Error", f"Failed to connect to {port}")
        return False

def send_command(command):
    global last_command_time
    current_time = time.time()

    if current_time - last_command_time < command_cooldown:
        return  # Skip sending command if in cooldown period

    try:
        with serial_lock:
            ser.write((command + '\n').encode())
            print(f"Sent: {command.strip()}")
            last_command_time = current_time  # Update last command time
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()}: {e}")

def convert_degrees_to_pulses(degrees):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def move_linear_stage(axis, direction, displacement):
    if ser is None:
        messagebox.showerror("Error", "Not connected to a device.")
        return

    print(f"Received axis: '{axis}', direction: '{direction}', displacement: {displacement}")

    valid_axes = ['X', 'Y', 'Z', 'T', 'r']
    if axis not in valid_axes or direction not in ['+', '-']:
        raise ValueError("Invalid axis or direction.")

    if axis == 'r':
        displacement = convert_degrees_to_pulses(displacement)

    command = f"{axis}{direction}{int(displacement)}\r"
    send_command(command)

def stop_motor_control():
    if ser:
        send_command("STOP")
    print("Motor control stopped.")

def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return (x + w // 2, y + h // 2), largest_contour, mask  # Return position, contour, and mask
    return None, None, mask  # Return mask even if no contours found

def move_motor_based_on_position(position, frame_center):
    if ser is None:
        return

    x, y = position
    frame_x, frame_y = frame_center

    hysteresis = 20

    if x < frame_x - hysteresis:
        send_command('right')
        print("Moving right")
    elif x > frame_x + hysteresis:
        send_command('left')
        print("Moving left")

    if y < frame_y - hysteresis:
        send_command('up')
        print("Moving up")
    elif y > frame_y + hysteresis:
        send_command('down')
        print("Moving down")

def nothing(x):
    pass

def create_hsv_calibration_window():
    global lower_bound, upper_bound
    cv2.namedWindow("HSV Calibration")

    cv2.createTrackbar("Lower Hue", "HSV Calibration", 0, 179, nothing)
    cv2.createTrackbar("Lower Sat", "HSV Calibration", 0, 255, nothing)
    cv2.createTrackbar("Lower Val", "HSV Calibration", 0, 255, nothing)
    cv2.createTrackbar("Upper Hue", "HSV Calibration", 179, 179, nothing)
    cv2.createTrackbar("Upper Sat", "HSV Calibration", 255, 255, nothing)
    cv2.createTrackbar("Upper Val", "HSV Calibration", 255, 255, nothing)

    while True:
        lower_hue = cv2.getTrackbarPos("Lower Hue", "HSV Calibration")
        lower_sat = cv2.getTrackbarPos("Lower Sat", "HSV Calibration")
        lower_val = cv2.getTrackbarPos("Lower Val", "HSV Calibration")
        upper_hue = cv2.getTrackbarPos("Upper Hue", "HSV Calibration")
        upper_sat = cv2.getTrackbarPos("Upper Sat", "HSV Calibration")
        upper_val = cv2.getTrackbarPos("Upper Val", "HSV Calibration")

        lower_bound = np.array([lower_hue, lower_sat, lower_val])
        upper_bound = np.array([upper_hue, upper_sat, upper_val])

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyWindow("HSV Calibration")

def launch_gui():
    def move_stage():
        try:
            axis = axis_entry.get().strip()
            direction = direction_var.get()
            displacement = int(displacement_entry.get().strip())
            move_linear_stage(axis, direction, displacement)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def connect():
        port = port_entry.get().strip()
        if connect_to_device(port):
            global camera_running
            camera_running = True

    root = tk.Tk()
    root.title("Motor Control Interface")

    tk.Label(root, text="COM Port:").pack()
    port_entry = tk.Entry(root)
    port_entry.pack()

    tk.Button(root, text="Connect", command=connect).pack(pady=5)

    tk.Label(root, text="Axis (X, Y, Z, T, r):").pack()
    axis_entry = tk.Entry(root)
    axis_entry.pack()

    direction_var = tk.StringVar(value='+')
    tk.Radiobutton(root, text='Positive (+)', variable=direction_var, value='+').pack()
    tk.Radiobutton(root, text='Negative (-)', variable=direction_var, value='-').pack()

    tk.Label(root, text="Displacement (in pulses or degrees for 'r'):").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    tk.Button(root, text="Move Stage", command=move_stage).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    root.mainloop()

def open_camera(camera_index):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from camera {camera_index}")
            break

        # Detect color and move motors if a position is found
        position, contour, mask = detect_color(frame)
        if position is not None:
            move_motor_based_on_position(position, (frame.shape[1] // 2, frame.shape[0] // 2))

        # Show the original frame with the blue dot
        if position is not None:
            cv2.circle(frame, position, 5, (255, 0, 0), -1)  # Draw the center point
        cv2.imshow(f"Camera {camera_index}", frame)

        # Show the masked image
        cv2.imshow(f"Mask {camera_index}", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Start the HSV calibration in a separate thread
    threading.Thread(target=create_hsv_calibration_window).start()

    # Start the GUI in a separate thread
    threading.Thread(target=launch_gui).start()

    camera_0_thread = threading.Thread(target=open_camera, args=(0,))
    camera_1_thread = threading.Thread(target=open_camera, args=(1,))
    camera_0_thread.start()
    camera_1_thread.start()

    camera_0_thread.join()
    camera_1_thread.join()
