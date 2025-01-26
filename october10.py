#import matlab.engine
import serial
import time
import threading
import cv2
import random
import numpy as np
import matplotlib.pyplot as plt

# settings
controller_port = 'COM5'
baud_rate = 9600
relay_state = False
motor_direction = False
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

#IMAGE RECOGNITION
def detect_rectangles_with_preprocessing(image, target_width=70, target_height=100, height_tolerance=20, width_tolerance=18):
    if image is None or image.size == 0:
        raise ValueError("Input image is empty or None.")
    
    height, width = image.shape[:2]
    kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharpened = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)

    # Define regions to blur (ensure they are within bounds)
    regions_to_blur = [
        (0, 0, 300, 195),
        (315, 195, 300, 195),
        (0,45,45,45)
        
    ]

    mask = np.ones(image.shape[:2], dtype=np.uint8) * 255
    for (x, y, w, h) in regions_to_blur:
        if x < 0 or y < 0 or x + w > width or y + h > height:
            continue
        region = sharpened[y:y+h, x:x+w]
        blurred_region = cv2.GaussianBlur(region, (25, 25), 0)
        sharpened[y:y+h, x:x+w] = blurred_region
        mask[y:y+h, x:x+w] = 0

    img_gray = cv2.cvtColor(sharpened, cv2.COLOR_BGR2GRAY)
    img_gray_masked = cv2.bitwise_and(img_gray, mask)

    # Thresholding
    _, binary = cv2.threshold(img_gray_masked, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    patch_image = np.copy(sharpened)
    rectangles = []
    centers = []

    for contour in contours:
        if cv2.contourArea(contour) >= 230:
            x, y, w, h = cv2.boundingRect(contour)
            rectangles.append((x, y, w, h))
            cv2.rectangle(patch_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Calculate and draw the center point
            center_x = x + w // 2
            center_y = y + h // 2
            centers.append((center_x, center_y))
            cv2.circle(patch_image, (center_x, center_y), 5, (0, 0, 255), -1)

    return sharpened, patch_image, len(rectangles), centers







def convert_degrees_to_pulses(degrees):
    stepper_angle_deg = 1.8
    transmission_ratio = 180
    subdivision = 2
    rotation_pulse_equivalent = stepper_angle_deg / (transmission_ratio * subdivision)
    pulses = round(degrees / rotation_pulse_equivalent, 0)
    print(f"Converting {degrees} degrees to {pulses} pulses.")
    return pulses
'''
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
    return command'''

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
        return None

    captured_image = None
    try:
        while True:
            ret, frame = camera.read()
            if not ret or frame is None:
                print(f"Failed to capture image from camera at index {camera_index}.")
                break

            cv2.imshow('Camera Feed', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                captured_image = frame
                print("Image captured.")
            elif key == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

    return captured_image

# Start camera feed and wait for the user to capture an image
captured_image = camera_feed()

if captured_image is not None:
    processed_image, patch_image, num_rectangles, centers = detect_rectangles_with_preprocessing(captured_image)

    # Display results
    plt.figure(figsize=(15, 5))
    
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB))
    plt.title('Processed Image')
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(patch_image, cv2.COLOR_BGR2RGB))
    plt.title('Detected Rectangles and Centers')
    plt.axis('off')

    plt.tight_layout()
    plt.show()
else:
    print("No image was captured.")


def main():
    captured_image=camera_feed()

    ser = connect_to_device(controller_port)
    if ser:
        try:
            print("success")
            if captured_image is not None:
                processed_image, patch_image, num_rectangles, centers = detect_rectangles_with_preprocessing(captured_image)

                # Display results
                plt.figure(figsize=(15, 5))
    
                plt.subplot(1, 2, 1)
                plt.imshow(cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB))
                plt.title('Processed Image')
                plt.axis('off')

                plt.subplot(1, 2, 2)
                plt.imshow(cv2.cvtColor(patch_image, cv2.COLOR_BGR2RGB))
                plt.title('Detected Rectangles and Centers')
                plt.axis('off')

                plt.tight_layout()
                plt.show()
            else:
                print("No image captured")
            motor_control(ser)

        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Failed to connect to the selected device.")

    
if __name__ == "__main__":
    main()
