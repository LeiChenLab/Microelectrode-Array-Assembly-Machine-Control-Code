import serial
import threading
import tkinter as tk
from tkinter import messagebox
import cv2
import time

# Global serial variable
ser = None
serial_lock = threading.Lock()
camera_running = False

# Command cooldown time
last_command_time = 0
command_cooldown = 0.5  # seconds

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
    """Move the motor along the specified axis with the given direction and displacement."""
    if ser is None:
        messagebox.showerror("Error", "Not connected to a device.")
        return

    print(f"Received axis: '{axis}', direction: '{direction}', displacement: {displacement}")

    valid_axes = ['X', 'Y', 'Z', 'T', 'r']  # Keep 'r' lowercase
    if axis not in valid_axes or direction not in ['+', '-']:
        raise ValueError("Invalid axis or direction.")

    # For rotary stage, convert displacement from degrees to pulses
    if axis == 'r':
        displacement = convert_degrees_to_pulses(displacement)

    command = f"{axis}{direction}{int(displacement)}\r"
    send_command(command)

def stop_motor_control():
    """Stop the motor control."""
    if ser:
        send_command("STOP")  # Assuming 'STOP' is a valid command for stopping the motor
    print("Motor control stopped.")

def detect_pcb(frame):
    """Detect PCB in the given frame using color filtering."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = (35, 100, 100)
    upper_green = (85, 255, 255)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return (x + w // 2, y + h // 2), largest_contour  # Return center of the PCB and contour
    return None, None

def move_motor_based_on_position(position, frame_center):
    """Move the motor based on the detected PCB position."""
    global ser
    if ser is None:
        return  # Do not move if not connected

    x, y = position
    frame_x, frame_y = frame_center

    # Define hysteresis values
    hysteresis = 20

    # Simple logic: If PCB is outside the hysteresis range, move the motor
    if x < frame_x - hysteresis:  # PCB is to the left
        send_command('right')
        print("Moving right")
    elif x > frame_x + hysteresis:  # PCB is to the right
        send_command('left')
        print("Moving left")

    if y < frame_y - hysteresis:  # PCB is above center
        send_command('up')
        print("Moving up")
    elif y > frame_y + hysteresis:  # PCB is below center
        send_command('down')
        print("Moving down")

def launch_gui():
    """Create and launch the main GUI."""
    def move_stage():
        """Move the motor stage with user inputs."""
        try:
            axis = axis_entry.get().strip()  # Keep the case as is
            direction = direction_var.get()
            displacement = int(displacement_entry.get().strip())
            move_linear_stage(axis, direction, displacement)
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def connect():
        """Connect to the specified COM port."""
        port = port_entry.get().strip()
        if connect_to_device(port):
            global camera_running
            camera_running = True  # Set camera_running to true when connected

    root = tk.Tk()
    root.title("Motor Control Interface")

    # COM Port Input
    tk.Label(root, text="COM Port:").pack()
    port_entry = tk.Entry(root)
    port_entry.pack()

    # Connect Button
    tk.Button(root, text="Connect", command=connect).pack(pady=5)

    # Axis Input
    tk.Label(root, text="Axis (X, Y, Z, T, r):").pack()
    axis_entry = tk.Entry(root)
    axis_entry.pack()

    # Direction Input
    direction_var = tk.StringVar(value='+')
    tk.Radiobutton(root, text='Positive (+)', variable=direction_var, value='+').pack()
    tk.Radiobutton(root, text='Negative (-)', variable=direction_var, value='-').pack()

    # Displacement Input
    tk.Label(root, text="Displacement (in pulses or degrees for 'r'):").pack()
    displacement_entry = tk.Entry(root)
    displacement_entry.pack()

    # Buttons for Control
    tk.Button(root, text="Move Stage", command=move_stage).pack(pady=10)
    tk.Button(root, text="Stop Motor Control", command=stop_motor_control).pack(pady=10)

    root.mainloop()
def open_camera(camera_index):
    """Function to open a camera feed."""
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

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Start the GUI in a separate thread
    threading.Thread(target=launch_gui).start()  
    camera_0_thread = threading.Thread(target=open_camera, args=(0,))
    camera_1_thread = threading.Thread(target=open_camera, args=(1,))
    camera_0_thread.start()
    camera_1_thread.start()


    camera_0_thread.join()
    camera_1_thread.join()
