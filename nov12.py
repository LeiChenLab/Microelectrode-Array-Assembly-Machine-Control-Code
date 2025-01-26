import serial
import threading
import tkinter as tk
from tkinter import messagebox
import cv2
import time
import matplotlib as plt
import serial.tools.list_ports
import keyboard  # Required for keyboard input detection

# Global serial variables for motor and relay
motor_ser = None
relay_ser = None
serial_lock = threading.Lock()
camera_running = False

# Command cooldown time
last_command_time = 0
command_cooldown = 0.001  # seconds

# Control settings
keyboard_control_enabled = False  # State of keyboard control mode
base_displacement = 15  # Default step size in degrees for all axes except 'r'
r_displacement = 0.25    # Default step size in degrees for 'r' axis

# Grid boundaries and resolution for each axis
grid_limits = {
    'X': {'min': -30000, 'max': 30000, 'step': 15},
    'Y': {'min': -30000, 'max': 30000, 'step': 15},
    'Z': {'min': -10000, 'max': 10000, 'step': 15},
    'r': {'min': -10000, 'max': 18000, 'step': 1},
    't': {'min': -30000, 'max': 30000, 'step': 15},
    'T': {'min': -10000, 'max': 10000, 'step': 15},
}

current_position = {'X': 0, 'Y': 0, 'Z': 0, 'r': 0,'t':0,'T':0}  # Initial position
origin_position = {'X': 0, 'Y': 0, 'Z': 0, 'r': 0,'t':0,'T':0}  # Origin position

# Axis controls
axis_controls = {
    'w': ('X', '+'),          # X axis positive
    's': ('X', '-'),          # X axis negative
    'a': ('Y', '+'),          # Y axis positive
    'd': ('Y', '-'),          # Y axis negative
    'shift': ('Z', '+'),      # Z axis positive
    'ctrl': ('Z', '-'),       # Z axis negative
    'e': ('r', '+'),          # r axis positive (rotary)
    'q': ('r', '-'),          # r axis negative (rotary)
    'z': ('t', '-'),          # T1 axis negative
    'x': ('t', '+'),          # T1 axis positive
    'r': ('T', '-'),          # T2 axis negative
    'f': ('T', '+')           # T2 axis positive
}




# Function to find the serial port
def find_port(device_description):
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if device_description in port.description:
            print(f"{device_description} detected on port {port.device}")
            return port.device
    print(f"No device with description '{device_description}' found.")
    return None

def auto_connect_ports():
    global motor_ser, relay_ser
    motor_port = find_port("USB-SERIAL CH340")
    relay_port = find_port("Arduino Uno")
    if motor_port:
        motor_ser = connect_to_device(motor_port, "Motor Controller")
    else:
        messagebox.showerror("Error", "Motor control device not found.")
    if relay_port:
        relay_ser = connect_to_device(relay_port, "Arduino Controller")
    else:
        messagebox.showerror("Error", "Relay device not found.")

def connect_to_device(port, device_name):
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        print(f"Connected successfully to {device_name} on {port}")
        if device_name == "Motor Controller":
            initialize_motor_controller(ser)  # Initialize motor controller on connection
        return ser
    except serial.SerialException as e:
        print(f"Connection error for {device_name} on {port}: {e}")
        messagebox.showerror("Error", f"Failed to connect to {device_name} on {port}")
        return None

def initialize_motor_controller(ser):
    try:
        print("Initializing motor controller...")
        response = send_command(ser, "?R\r", "Motor Controller")
        
        if response == "?R":
            print("Motor controller initialized successfully.")
        else:
            print(f"Motor controller initialization failed. Response: {response}")
    except Exception as e:
        print(f"Error during motor controller initialization: {e}")

def send_command(ser, command, device_name):
    global last_command_time
    current_time = time.time()
    if current_time - last_command_time < command_cooldown:
        print(f"{device_name} command cooldown active. Skipping command.")
        return
    try:
        with serial_lock:
            ser.write((command + '\n').encode())
            print(f"Sent command to {device_name}: {command.strip()}")
            last_command_time = current_time  # Update last command time
            time.sleep(0.009)  # Give the device time to respond
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode().strip()
                print(f"{device_name} response: {response}")
                handle_motor_response(response)
                return response
            else:
                print(f"No response from {device_name}.")
                return None
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()} to {device_name}: {e}")
        return None

def handle_motor_response(response):
    if "ERR" in response:
        print("Error received from Motor Controller. Check motor configuration or command syntax.")
    else:
        print(f"Motor Controller response: {response}")

def convert_degrees_to_pulses(degrees):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses

def set_origin():
    global origin_position
    origin_position = current_position.copy()
    print(f"Origin set to position: X={origin_position['X']}, Y={origin_position['Y']}, Z={origin_position['Z']}, r={origin_position['r']}")

def return_to_origin():
    """Move to origin one axis at a time: r, y, x, z."""
    move_to_coordinate(None, None, None, origin_position['r'])
    move_to_coordinate(None, origin_position['Y'], None, None)
    move_to_coordinate(origin_position['X'], None, None, None)
    move_to_coordinate(None, None, origin_position['Z'], None)

def move_to_coordinate(x=None, y=None, z=None, r=None,t=None):
    global current_position
    def calculate_steps(axis, target):
        if axis in grid_limits:
            step_size = grid_limits[axis]['step']
            delta = (target - current_position[axis]) / step_size
            return int(delta * step_size)
        return 0
    if r is not None:
        r_steps = calculate_steps('r', r)
        if r_steps:
            move_linear_stage('r', '+' if r_steps > 0 else '-', abs(r_steps))
        current_position['r'] = r
        print(f"Moved to position: r={r}")
    if y is not None:
        y_steps = calculate_steps('Y', y)
        if y_steps:
            move_linear_stage('Y', '+' if y_steps > 0 else '-', abs(y_steps))
        current_position['Y'] = y
        print(f"Moved to position: Y={y}")
    if x is not None:
        x_steps = calculate_steps('X', x)
        if x_steps:
            move_linear_stage('X', '+' if x_steps > 0 else '-', abs(x_steps))
        current_position['X'] = x
        print(f"Moved to position: X={x}")
    if z is not None:
        z_steps = calculate_steps('Z', z)
        if z_steps:
            move_linear_stage('Z', '+' if z_steps > 0 else '-', abs(z_steps))
        current_position['Z'] = z
        print(f"Moved to position: Z={z}")
    if t is not None:
        t_steps = calculate_steps('t', z)
        if z_steps:
            move_linear_stage('t', '+' if t_steps > 0 else '-', abs(t_steps))
        current_position['t'] = t
        print(f"Moved to position: t={t}")

def move_linear_stage(axis, direction, displacement):
    if motor_ser is None:
        messagebox.showerror("Error", "Not connected to motor control device.")
        return
    print(f"Received axis: '{axis}', direction: '{direction}', displacement: {displacement}")
    valid_axes = ['X', 'Y', 'Z', 'r', 't', 'T']
    if axis not in valid_axes or direction not in ['+', '-']:
        messagebox.showerror("Error", "Invalid axis or direction.")
        return
    if axis == 'r':
        displacement = convert_degrees_to_pulses(displacement)
    command = f"{axis}{direction}{int(displacement)}\r"
    send_command(motor_ser, command, "Motor Controller")

def stop_motor_control():
    if motor_ser:
        send_command(motor_ser, "STOP", "Motor Controller")
    else:
        messagebox.showerror("Error", "Not connected to motor control device.")
    print("Motor control stopped.")

def continuous_motor_control():
    global keyboard_control_enabled
    while True:
        if keyboard_control_enabled:
            for key, (axis, direction) in axis_controls.items():
                if keyboard.is_pressed(key):
                    step_size = r_displacement if axis == 'r' else base_displacement
                    if keyboard.is_pressed("v"):
                        step_size *= 2
                    if keyboard.is_pressed("space"):
                        step_size /= 2
                    move_linear_stage(axis, direction, step_size)
                    
                    if direction == '+':
                        current_position[axis] += step_size
                    else:
                        current_position[axis] -= step_size
                    
                    current_position[axis] = max(grid_limits[axis]['min'], min(current_position[axis], grid_limits[axis]['max']))
                    print(f"Updated position on {axis} axis: {current_position[axis]}")
        time.sleep(0.1)

def laser_relay_on():
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_On", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def laser_relay_off():
    if relay_ser:
        send_command(relay_ser, "Laser_Relay_Off", "Relay Controller")
    else:
        messagebox.showerror("Error", "Not connected to relay device.")

def toggle_keyboard_control():
    global keyboard_control_enabled
    keyboard_control_enabled = not keyboard_control_enabled
    status = "enabled" if keyboard_control_enabled else "disabled"
    print(f"Keyboard motor control {status}.")

def launch_gui():
    global motor_port_entry, relay_port_entry
    def move_stage():
        try:
            axis = axis_entry.get().strip()
            direction = direction_var.get()
            displacement = int(displacement_entry.get().strip())
            move_linear_stage(axis, direction, displacement)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def move_to_grid_position():
        try:
            x = int(x_entry.get().strip())
            y = int(y_entry.get().strip())
            z = int(z_entry.get().strip())
            r = int(r_entry.get().strip())
            t = int(r_entry.get().strip())
            T = int(r_entry.get().strip())
            move_to_coordinate(x, y, z, r,t,T)
        except ValueError:
            messagebox.showerror("Error", "Invalid grid coordinate input.")

    root = tk.Tk()
    root.title("Motor Control and Camera Feed")
    tk.Label(root, text="Motor Port:").pack()
    motor_port_entry = tk.Entry(root)
    motor_port_entry.pack()
    tk.Label(root, text="Arduino Port:").pack()
    relay_port_entry = tk.Entry(root)
    relay_port_entry.pack()
    
    tk.Label(root, text="Axis (X, Y, Z, r, t, T):").pack()
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
    tk.Label(root, text="Grid Coordinates:").pack()
    tk.Label(root, text="X:").pack()
    x_entry = tk.Entry(root)
    x_entry.pack()
    tk.Label(root, text="Y:").pack()
    y_entry = tk.Entry(root)
    y_entry.pack()
    tk.Label(root, text="Z:").pack()
    z_entry = tk.Entry(root)
    z_entry.pack()
    tk.Label(root, text="R:").pack()
    r_entry = tk.Entry(root)
    r_entry.pack()
    tk.Button(root, text="Move to Coordinate", command=move_to_grid_position).pack(pady= 10)
    tk.Button(root, text="Set Origin", command=set_origin).pack(pady=5)
    tk.Button(root, text="Return to Origin", command=return_to_origin).pack(pady=5)
    keyboard_control_var = tk.IntVar(value=0)
    tk.Checkbutton(root, text="Keyboard Movement Mode", variable=keyboard_control_var, command=toggle_keyboard_control).pack(pady=5)
    tk.Button(root, text="Laser Relay On", command=laser_relay_on).pack(pady=5)
    tk.Button(root, text="Laser Relay Off", command=laser_relay_off).pack(pady=5)
    root.mainloop()

def detect_cameras():
    """Detect available cameras by trying different indices."""
    available_cameras = []
    index = 0
    while True:
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()  # Release the camera after detecting it
        else:
            break
        index += 1
    if available_cameras:
        print(f"Detected cameras at indices: {available_cameras}")
    else:
        print("No cameras detected.")
    return available_cameras


import numpy as np

def detect_wires_and_patches(image):
    if image is None or image.size == 0:
        raise ValueError("Input image is empty or None.")

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold the image to keep dark (black) areas, ignoring bright areas
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)  # Inverse threshold: Dark areas become white

    # Canny edge detection on the blurred image
    edges = cv2.Canny(blurred, 50, 150)  # Adjust these thresholds as necessary

    # Apply morphological dilation to improve contour detection
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilated = cv2.dilate(edges, kernel, iterations=1)

    # Find contours from the dilated edges
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print(f"Number of contours detected: {len(contours)}")

    wire_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= 75:  # Adjust this threshold based on wire size
            # Filter by aspect ratio (long and thin contours)
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            if aspect_ratio > 4:  # Adjust aspect ratio threshold for wire-like shapes
                wire_contours.append(contour)

    # If no wire contours are found, return the original image
    if not wire_contours:
        return image, 0

    # Find the longest wire contour (the one with the largest width or aspect ratio)
    longest_wire = max(wire_contours, key=lambda contour: cv2.boundingRect(contour)[2])  # Compare by width

    # Draw only the longest wire in green
    output_image = image.copy()
    cv2.drawContours(output_image, [longest_wire], -1, (0, 255, 0), 2)  # Green color

    # Get the convex hull of the longest wire
    hull = cv2.convexHull(longest_wire)

    # Get the extreme points of the hull
    extreme_points = get_extreme_points(hull)

    # Draw the red point at the leftmost extreme point (the first point)
    leftmost_point = extreme_points[0]  # The leftmost point is the first one
    cv2.circle(output_image, leftmost_point, 5, (0, 0, 255), -1)  # Red color for extreme points

    # Calculate the centroid of the contour (not drawn, as per your request)
    M = cv2.moments(longest_wire)
    if M['m00'] != 0:  # Avoid division by zero
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        # The centroid (cx, cy) is calculated but not drawn

    # Now, detect patches (rectangles) in the image
    rectangles = detect_rectangles(image)

    # Check if the leftmost point touches any of the rectangles
    for rect in rectangles:
        x, y, w, h = rect  # Unpack the rectangle (x, y, width, height)
        if is_point_near_rectangle(leftmost_point, (x, y, w, h)):
            cv2.rectangle(output_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Draw rectangle in blue
            cv2.putText(output_image, 'Leftmost point near patch', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    # Return the processed image and the number of wires (just 1, the longest one)
    return output_image, 1

def detect_rectangles(image):
    """Detect all rectangles in the image (as patches)."""
    height, width = image.shape[:2]
    kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])  # Sharpening kernel
    sharpened = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)

    proces = np.ones(image.shape[:2], dtype=np.uint8) * 255
    img_gray = cv2.cvtColor(sharpened, cv2.COLOR_BGR2GRAY)
    img_gray_procesed = cv2.bitwise_and(img_gray, proces)

    # Thresholding
    _, binary = cv2.threshold(img_gray_procesed, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    patch_image = np.copy(sharpened)
    rectangles = []
    centers = []
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        if cv2.contourArea(contour) >= 230:  # Filter based on contour area
            x, y, w, h = cv2.boundingRect(contour)
            rectangles.append((x, y, w, h))
            cv2.rectangle(patch_image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw green rectangle

            # Calculate and draw the center point
            center_x = x + w // 2
            center_y = y + h // 2
            centers.append((center_x, center_y))
            cv2.circle(patch_image, (center_x, center_y), 5, (0, 0, 255), -1)

    return rectangles

def is_point_near_rectangle(point, rectangle, tolerance=10):
    """Check if a point is near a rectangle (within a tolerance distance)."""
    x, y, w, h = rectangle
    px, py = point

    # Check if the point is within the rectangle's bounds plus a tolerance
    if x - tolerance <= px <= x + w + tolerance and y - tolerance <= py <= y + h + tolerance:
        return True
    return False

def get_extreme_points(hull):
    """Find extreme points in the convex hull: top, bottom, left, right."""
    leftmost = tuple(hull[hull[:, :, 0].argmin()][0])  # Leftmost point
    rightmost = tuple(hull[hull[:, :, 0].argmax()][0])  # Rightmost point
    topmost = tuple(hull[hull[:, :, 1].argmin()][0])  # Topmost point
    bottommost = tuple(hull[hull[:, :, 1].argmax()][0])  # Bottommost point

    # Return the extreme points as leftmost, rightmost, topmost, and bottommost
    return [leftmost, rightmost, topmost, bottommost]
import cv2
import numpy as np

def detect_rectangles_with_preprocessing(image, target_width=70, target_height=100, height_tolerance=20, width_tolerance=18):
    height, width = image.shape[:2]
    
    # Convert image to HSV and detect silver regions
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_silver = np.array([0, 0, 180])  # Adjusted to include a broader range of silver
    upper_silver = np.array([180, 20, 255])  # Adjusted range
    silver_mask = cv2.inRange(hsv, lower_silver, upper_silver)
    silver_regions = cv2.bitwise_and(image, image, mask=silver_mask)

    # Convert to grayscale and apply Gaussian blur to reduce noise
    gray = cv2.cvtColor(silver_regions, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Perform Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Apply dilation to close gaps in edges
    kernel = np.ones((5, 5), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=2)

    # Apply histogram equalization to enhance the image
    equalized = cv2.equalizeHist(blurred)

    # Apply adaptive thresholding
    thresh = cv2.adaptiveThreshold(equalized, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # Use the dilated image for contour detection
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create an empty image to draw rectangles on
    patch_image = np.copy(image)

    rectangles = []
    centers = []
    min_area = 230  # Minimum area for rectangles to be considered

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Get bounding rectangle for the contour
        x, y, w, h = cv2.boundingRect(contour)

        # Filter based on size and position
        if len(approx) == 4 and y > height * 0.75 and y < height * 0.90:  # Filter based on position and shape
            if (abs(w - target_width) <= width_tolerance and abs(h - target_height) <= height_tolerance) or cv2.contourArea(contour) >= min_area:
                rectangles.append((x, y, w, h))
                cv2.rectangle(patch_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate and draw the center of the rectangle
                center_x = x + w // 2
                center_y = y + h // 2
                centers.append((center_x, center_y))
                cv2.circle(patch_image, (center_x, center_y), 5, (0, 0, 255), -1)

    # Show the final image with rectangles for debugging
    cv2.imshow('Detected Rectangles', patch_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return patch_image, len(rectangles)



def open_camera(camera_index):
    """Open the camera with the given index and display video feed."""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from camera {camera_index}")
            break
        cv2.imshow(f"Camera {camera_index}", frame)
        key = cv2.waitKey(1)  # Check for key press
        if key & 0xFF == ord('q'):
            break
        if key & 0xFF == ord('c'):  # Check if 'c' key is pressed
            detected_image, num_wires = detect_wires_and_patches(frame)
            cv2.imshow('Detected Wires', detected_image)
            print(f"Number of wires detected: {num_wires}")
        if key & 0xFF == ord('s'):  # Check if 's' key is pressed
            detected_rectangle, num_rectangle = detect_rectangles_with_preprocessing(frame)
            cv2.imshow('Detected Patches', detected_rectangle)
            print(f"Number of patches detected: {num_rectangle}")
    cap.release()
    cv2.destroyAllWindows()

def start_multiple_cameras():
    """Start camera capture on available cameras."""
    available_cameras = detect_cameras()
    if available_cameras:
        threads = []
        for camera_index in available_cameras:
            camera_thread = threading.Thread(target=open_camera, args=(camera_index,))
            threads.append(camera_thread)
            camera_thread.start()
        # Join threads to ensure all cameras are processed
        for thread in threads:
            thread.join()
    else:
        messagebox.showerror("Error", "No cameras detected.")

if __name__ == "__main__":
    threading.Thread(target=launch_gui).start()
    threading.Thread(target=continuous_motor_control).start()
    auto_connect_ports()  # Automatically connect on program start
    threading.Thread(target=start_multiple_cameras).start() 
    