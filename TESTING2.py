import serial
import threading
import tkinter as tk
from tkinter import messagebox
import cv2
import time
import serial.tools.list_ports
import keyboard  # Required for keyboard input detection

# Global serial variables for motor and relay
motor_ser = None
relay_ser = None
serial_lock = threading.Lock()
camera_running = False

# Global variable to track the first connection to the relay
first_relay_connection = True

# Command cooldown time
last_command_time = 0
command_cooldown = 0.001  # seconds

# Control settings
keyboard_control_enabled = False  # State of keyboard control mode
base_displacement = 15  # Default step size in degrees for all axes except 'r'
r_displacement = 0.25    # Default step size in degrees for 'r' axis

# Grid boundaries and resolution for each axis, allowing negative side
# Updated grid_limits to include 'T' and 't'
grid_limits = {
    'X': {'min': -30000, 'max': 30000, 'step': 15},
    'Y': {'min': -30000, 'max': 30000, 'step': 15},
    'Z': {'min': -10000, 'max': 10000, 'step': 15},
    'r': {'min': -10000, 'max': 18000, 'step': 1},
    'T': {'min': -10000, 'max': 10000, 'step': 15},  # Added 'T'
    't': {'min': -10000, 'max': 10000, 'step': 15}   # Added 't'
}

# Updated current_position to include 'T' and 't'
current_position = {'X': 0, 'Y': 0, 'Z': 0, 'r': 0, 'T': 0, 't': 0}  # Added 'T' and 't' # Initial position
origin_position = {'X': 0, 'Y': 0, 'Z': 0, 'r': 0}  # Origin position

# Axis controls
axis_controls = {
    'w': ('X', '+'),          # X axis positive
    's': ('X', '-'),          # X axis negative
    'a': ('Y', '+'),          # Y axis positive
    'd': ('Y', '-'),          # Y axis negative
    'shift': ('Z', '+'),      # Z axis positive
    'ctrl': ('Z', '-'),       # Z axis negative (changed from 'caps lock' to 'ctrl')
    'e': ('r', '+'),          # r axis positive (rotary)
    'q': ('r', '-'),          # r axis negative (rotary)
    'z': ('t', '-'),          # T1 axis negative
    'x': ('t', '+'),          # T1 axis positive
    'r': ('T', '-'),          # T2 axis negative
    'f': ('T', '+')           # T2 axis positive
}

def find_port(device_description):
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if device_description in port.description:
            print(f"{device_description} detected on port {port.device}")
            return port.device
    print(f"No device with description '{device_description}' found.")
    return None

def auto_connect_ports():
    global motor_port_entry, relay_port_entry, motor_ser, relay_ser

    motor_port = find_port("USB-SERIAL CH340")
    relay_port = find_port("Arduino Uno")

    if motor_port:
        motor_port_entry.delete(0, tk.END)
        motor_port_entry.insert(0, motor_port)
        motor_ser = connect_to_device(motor_port, "Motor Controller")
    else:
        messagebox.showerror("Error", "Motor control device not found.")

    if relay_port:
        relay_port_entry.delete(0, tk.END)
        relay_port_entry.insert(0, relay_port)
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
            time.sleep(0.1)  # Give the device time to respond

            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode().strip()
                print(f"{device_name} response: {response}")
                #handle_motor_response(response)
                if "ERR" in response:
                    print(f"Boundary or other error detected for {device_name}.")
                return response
            else:
                print(f"No response from {device_name}.")
                return None
    except serial.SerialException as e:
        print(f"Failed to send command {command.strip()} to {device_name}: {e}")
        return None

#def handle_motor_response(response):
 #   if "ERR" in response:
#        print("Error received from Motor Controller. Check motor configuration or command syntax.")
#    else:
#        print(f"Motor Controller response: {response}")

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

def move_to_coordinate(x=None, y=None, z=None, r=None):
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
    response = send_command(motor_ser, command, "Motor Controller")
    if response and "ERR" in response:
        print(f"Boundary hit on axis {axis} in direction {direction}. No further movement.")
        Ratbrain
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

                    # Update current_position for valid axes
                    if axis in current_position:
                        if direction == '+':
                            current_position[axis] += step_size
                        else:
                            current_position[axis] -= step_size

                        # Enforce grid limits
                        current_position[axis] = max(grid_limits[axis]['min'], min(current_position[axis], grid_limits[axis]['max']))
                        print(f"Updated position on {axis} axis: {current_position[axis]}")
        time.sleep(0.2)
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
            move_to_coordinate(x, y, z, r)
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

    tk.Button(root, text="Move to Coordinate", command=move_to_grid_position).pack(pady=10)
    tk.Button(root, text="Set Origin", command=set_origin).pack(pady=5)
    tk.Button(root, text="Return to Origin", command=return_to_origin).pack(pady=5)
    
    keyboard_control_var = tk.IntVar(value=0)
    tk.Checkbutton(root, text="Keyboard Movement Mode", variable=keyboard_control_var, command=toggle_keyboard_control).pack(pady=5)

    tk.Button(root, text="Laser Relay On", command=laser_relay_on).pack(pady=5)
    tk.Button(root, text="Laser Relay Off", command=laser_relay_off).pack(pady=5)

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

        cv2.imshow(f"Camera {camera_index}", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    threading.Thread(target=launch_gui).start()
    threading.Thread(target=continuous_motor_control).start()

    auto_connect_ports()  # Automatically connect on program start

    camera_0_thread = threading.Thread(target=open_camera, args=(0,))
    camera_1_thread = threading.Thread(target=open_camera, args=(1,))
    camera_0_thread.start()
    camera_1_thread.start()

    camera_0_thread.join()
    camera_1_thread.join()
