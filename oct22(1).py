import serial
import threading
import tkinter as tk
from tkinter import messagebox

# Global serial variable
ser = None
serial_lock = threading.Lock()

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
    try:
        with serial_lock:
            ser.write((command + '\n').encode())
            print(f"Sent: {command.strip()}")
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
        connect_to_device(port)

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

if __name__ == "__main__":
    launch_gui()
