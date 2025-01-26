import serial
import time
import logging


# Set up logging
logging.basicConfig(level=logging.INFO)
def send_command(ser, command):
    try:
        ser.write(f"{command}\n".encode())
        time.sleep(5.0)  # Increased time for command processing
        response = read_response(ser)
        logging.info(f"Sent command: {command}")
        logging.info(f"Raw response: '{response}' (length: {len(response)})")  # Log response length
        
        if is_error_response(response):
            logging.error(f"Error encountered with command '{command}': {response}")
            return ""
        
        return response
    except Exception as e:
        logging.error(f"Error sending command '{command}': {e}")
        return ""


def read_response(ser):
    response = ""
    start_time = time.time()
    while time.time() - start_time < 10:  # 5-second timeout
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            response += line + "\n"
            if "Enter target speed in RPM followed by target position in revolutions:" in line:
                break
        time.sleep(0.01)
    return response.strip()

def connect_to_device(port, baud_rate=9600, timeout=1):
    try:
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        time.sleep(2)  # Wait for the connection to establish
        logging.info(f"Connected successfully to {port}")
        return ser
    except serial.SerialException as e:
        logging.error(f"Failed to connect to {port}: {e}")
        return None

def is_error_response(response):
    error_indicators = ["error", "failed", "invalid"]
    return any(indicator in response.lower() for indicator in error_indicators)

def main():
    port = 'COM4'
    ser = connect_to_device(port)
    
    if ser:
        try:
            commands = [
                ("STOP", "Stopping any ongoing operation"),
                ("100", "Setting target speed to 100 RPM"),
                ("5.0", "Setting target position to 5.0 revolutions")
            ]

            for command, description in commands:
                logging.info(f"\n{description}")
                response = send_command(ser, command)
                logging.info(f"Cleaned response: '{response}'")

                if is_error_response(response):
                    logging.error(f"Error encountered with command '{command}'. Aborting sequence.")
                    break

                if "Motor stopped" in response:
                    logging.info("Motor successfully stopped.")
                elif "Enter target speed in RPM followed by target position in revolutions:" in response:
                    logging.info("Arduino is ready for the next command.")
                else:
                    logging.warning("Command executed, but unexpected response. Continuing sequence.")

                time.sleep(1)  # Wait between commands
        
        except Exception as e:
            logging.error(f"An error occurred: {e}")
        
        finally:
            ser.close()
            logging.info("Serial connection closed.")
    else:
        logging.error(f"Could not open port {port}")

if __name__ == "__main__":
    main()
