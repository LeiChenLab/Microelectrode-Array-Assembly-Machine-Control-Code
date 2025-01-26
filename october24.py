import serial
import time
import tkinter as tk
from tkinter import messagebox

# Serial port settings
controller_port = 'COM4'  # Adjust the serial port as needed
baud_rate = 9600
arduino_port = 'COM5'

relay_state = False
motor_direction = False

# Motor control variables
target_position_counts = 0
target_speed_rpm = 0
gear_ratio = 46.85
encoder_cpr = 48
output_cpr = encoder_cpr * gear_ratio

# Global serial object
ser = None

def send_command(command):
    global ser
    try:
        ser.write((command + '\n').encode())
        print(f"Sent command: {command.strip()}")

        # Wait for a response
        time.sleep(1)
        response = ser.read(ser.in_waiting).decode()
        cleaned_response = response.replace('\r', '').replace('\n', '').strip()
        print(f"Response: '{cleaned_response}'")

        return cleaned_response
    except serial.SerialException as e:
        messagebox.showerror("Error", f"Failed to send command {command.strip()}: {e}")
        return None

def connect_to_device(port):
    global ser
    try:
        ser = serial.Serial(port, baud_rate)
        print(f"Connected successfully to device on {port}")
        return True
    except serial.SerialException as e:
        messagebox.showerror("Connection Error", f"SerialException for port {port}: {e}")
        return False

def toggle_relay():
    global relay_state
    relay_state = not relay_state
    command = "Laser_Relay_On" if relay_state else "Laser_Relay_Off"
    response = send_command(command)
    if response:
        print(f"Relay toggled: {command}")

def change_motor_direction():
    global motor_direction
    motor_direction = not motor_direction
    direction_command = "F\n" if motor_direction else "B\n"
    send_command(direction_command)
    direction = "FORWARD" if motor_direction else "BACKWARD"
    print(f"Motor running {direction}")

def control_motor():
    global target_speed_rpm, target_position_counts
    try:
        target_speed_rpm = int(speed_entry.get())
        target_position_revs = float(position_entry.get())
        target_position_counts = int(target_position_revs * output_cpr)
        print(f"Target speed set to: {target_speed_rpm} RPM, Target position set to: {target_position_counts} counts.")
        
        send_command('R')  # Command to read current encoder counts
        time.sleep(0.5)  # Increased delay
        response = ser.readline().decode().strip()
        if response.startswith("ENCODER_COUNT "):
            encoder_count = int(response.split()[1])
            error = target_position_counts - encoder_count
            print(f"Current counts: {encoder_count}, Target counts: {target_position_counts}, Error: {error}")
            if abs(error) > 10:
                speed_pwm = int(map_value(target_speed_rpm, 0, 300, 0, 255))
                send_command(f"S{speed_pwm}")
                send_command('F' if error > 0 else 'B')
            else:
                send_command("STOP")
                print("Target position reached.")
        else:
            print(f"Unexpected response: {response}")
    except ValueError as e:
        messagebox.showerror("Input Error", f"Invalid input: {e}")

def extruder_motor():
    try:
        position = int(extruder_entry.get())
        send_command(f"Extruder_Motor {position}")
    except ValueError as e:
        messagebox.showerror("Input Error", f"Invalid extruder position: {e}")

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def create_gui():
    global speed_entry, position_entry, extruder_entry

    root = tk.Tk()
    root.title("Arduino Control")

    tk.Button(root, text="Toggle Relay", command=toggle_relay).pack(pady=5)
    tk.Button(root, text="Change Motor Direction", command=change_motor_direction).pack(pady=5)

    tk.Label(root, text="Target Speed (RPM):").pack()
    speed_entry = tk.Entry(root)
    speed_entry.pack(pady=5)

    tk.Label(root, text="Target Position (Revolutions):").pack()
    position_entry = tk.Entry(root)
    position_entry.pack(pady=5)

    tk.Button(root, text="Control Motor", command=control_motor).pack(pady=5)

    tk.Label(root, text="Extruder Motor Position:").pack()
    extruder_entry = tk.Entry(root)
    extruder_entry.pack(pady=5)

    tk.Button(root, text="Set Extruder Position", command=extruder_motor).pack(pady=5)

    root.mainloop()

if __name__ == "__main__":
    if connect_to_device(arduino_port):
        create_gui()
    else:
        print("Failed to connect to device.")
