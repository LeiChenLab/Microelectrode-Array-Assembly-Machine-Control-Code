'''import serial
import time

# Serial port settings
serial_port = '/dev/ttyUSB0'  # Adjust the serial port as needed
baud_rate = 9600

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

def connect_to_controller(ser):
    try:
        response = send_command(ser, "?R\r")
        if response:
            expected_response = "?ROK"
            if response == expected_response:
                print(f"Connected successfully to motion controller on {serial_port}")
                return True
            else:
                print(f"Failed to connect on {serial_port}, response: '{response}' (expected '{expected_response}')")
                return False
    except serial.SerialException as e:
        print(f"SerialException for port {serial_port}: {e}")
    return False

def connect_to_arduino(ser):
    try:
        ser.write(b'\n')  # Send a newline to wake up the Arduino
        print(f"Connected successfully to Arduino on {serial_port}")
        return True
    except serial.SerialException as e:
        print(f"SerialException for port {serial_port}: {e}")
        return False

def move_stage(ser, axis, direction, displacement):
    if direction not in ['+', '-']:
        raise ValueError("Invalid direction specified. Choose '+' for positive or '-' for negative direction.")
    valid_axes = ['X', 'Y', 'Z', 'r', 't', 'T']
    if axis not in valid_axes:
        raise ValueError("Invalid axis specified. Choose from 'X', 'Y', 'Z', 'r', 't', 'T'.")
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
    current_counts = int(ser.readline().decode().strip())  # Simulating reading encoder counts from serial
    error = position_counts - current_counts
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

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main():
    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Serial port {serial_port} opened.")

        device = input("Enter 'arduino' to connect to Arduino or 'controller' to connect to Motion Controller: ").strip().lower()

        if device == 'arduino':
            if connect_to_arduino(ser):
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

                    else:
                        print("Invalid command. Please enter 't', 'd', 'm', 'e', or 'exit'.")

                    time.sleep(0.5)
        elif device == 'controller':
            if connect_to_controller(ser):
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
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()'''



















import serial
import time

# Serial port settings
serial_port = 'COM4'  # Adjust the serial port as needed
baud_rate = 9600

relay_state = False
motor_direction = False

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

def connect_to_arduino(ser):
    try:
        ser.write(b'\n')  # Send a newline to wake up the Arduino
        print(f"Connected successfully to Arduino on {serial_port}")
        return True
    except serial.SerialException as e:
        print(f"SerialException for port {serial_port}: {e}")
        return False

def toggle_relay(ser):
    global relay_state
    relay_state = not relay_state
    command = "t"  # 't' is the command to toggle relay
    response = send_command(ser, command)
    return response

def change_motor_direction(ser):
    global motor_direction
    motor_direction = not motor_direction
    command = "d"  # 'd' is the command to change motor direction
    response = send_command(ser, command)
    return response

def main():
    try:
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Serial port {serial_port} opened.")

        if connect_to_arduino(ser):
            print("Enter 't' to toggle relay or 'd' to change motor direction")
            
            while True:
                command = input("Enter command: ").strip().lower()

                if command == 't':
                    toggle_relay(ser)
                    
                elif command == 'd':
                    change_motor_direction(ser)

                elif command == 'exit':
                    break

                else:
                    print("Invalid command. Please enter 't', 'd', or 'exit'.")

                time.sleep(0.5)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
