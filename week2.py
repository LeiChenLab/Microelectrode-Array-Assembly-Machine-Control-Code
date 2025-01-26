import serial
import time

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
    valid_axes = ['X', 'Y', 'Z', 'R', 't', 'T']
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'R', 't', 'T'.")
    command = f"{axis}{direction}{displacement}\r"
    ser.write(command.encode())
    print(f"Command sent to move stage: {command.strip()}")

def toggle_relay(ser):
    global relay_state
    relay_state = not relay_state
    command = "Laser_Relay_On" if relay_state else "Laser_Relay_Off"
    response = send_command(ser, command)
    return response

def change_motor_direction():
    global motor_direction
    motor_direction = not motor_direction
    direction = "FORWARD" if motor_direction else "BACKWARD"
    print(f"Motor running {direction}")

def extruder_motor(ser, position):
    command = f"Extruder_Motor {position}"
    response = send_command(ser, command)
    return response

def control_motor(ser, speed_rpm, position_counts):
    try:
        ser.write(b'R\n')  # Command to read current encoder counts
        time.sleep(0.5)  # Increased delay
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Encoder response: {response}")
            if response.startswith("ENCODER_COUNT "):
                encoder_count = int(response.split()[1])
                error = position_counts - encoder_count
                print(f"Current counts: {encoder_count}, Target counts: {position_counts}, Error: {error}")
                if abs(error) > 10:
                    speed_pwm = int(map_value(speed_rpm, 0, 300, 0, 255))
                    ser.write(f"S{speed_pwm}\n".encode())
                    if error > 0:
                        ser.write(b'F\n')  # Forward
                    else:
                        ser.write(b'B\n')  # Backward
                else:
                    ser.write(b'STOP\n')  # Stop motor
                    print("Target position reached.")
            else:
                print(f"Unexpected response: {response}")
        else:
            print("No response received from encoder.")
    except ValueError as e:
        print(f"Error reading encoder counts: {e}")

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main():
    try:
        device = input("Enter 'arduino' to connect to Arduino or 'controller' to connect to Motion Controller: ").strip().lower()

        if device == 'arduino':
            ser = connect_to_device(arduino_port)
            if ser:
                print("Enter 't' to toggle relay, 'd' to change motor direction, 'm' for motor control, or 'e' to set extruder motor position")
                
                while True:
                    command = input("Enter command: ").strip().lower()

                    if command == 't':
                        toggle_relay(ser)
                        
                    elif command == 'd':
                        change_motor_direction()

                    elif command == 'm':
                        global target_speed_rpm
                        global target_position_counts
                        
                        target_speed_rpm = int(input("Enter target speed in RPM: "))
                        target_position_revs = float(input("Enter target position in revolutions: "))
                        target_position_counts = int(target_position_revs * output_cpr)
                        print(f"Target speed set to: {target_speed_rpm} RPM, Target position set to: {target_position_counts} counts.")
                        
                        control_motor(ser, target_speed_rpm, target_position_counts)

                    elif command == 'e':
                        extruder_position = int(input("Enter target position for extruder motor: "))
                        extruder_motor(ser, extruder_position)

                    elif command == 'exit':
                        break

                    elif command == 'test':
                        # Test command to verify encoder response
                        response = send_command(ser, 'R')
                        print(f"Test command response: {response}")

                    else:
                        print("Invalid command. Please enter 't', 'd', 'm', 'e', 'test', or 'exit'.")

                    time.sleep(0.5)

        elif device == 'controller':
            ser = connect_to_device(controller_port)
            if ser:
                while True:
                    command = input("Enter 'move' to move the stage or 'exit' to quit: ").strip().lower()

                    if command == 'exit':
                        break

                    if command == 'move':
                        axis = input("Enter axis (X, Y, Z, r, t, T): ").strip().upper()
                        direction = input("Enter direction ('+' for positive or '-' for negative): ").strip()
                        displacement = input("Enter displacement (pulse number): ").strip()

                        try:
                            displacement = int(displacement)
                            move_stage(ser, axis, direction, displacement)
                        except ValueError as e:
                            print(f"Invalid input: {e}")

                    else:
                        print("Invalid command. Please enter 'move' or 'exit'.")

                    time.sleep(0.5)

        else:
            print("Invalid device selection.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
