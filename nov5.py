import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_gold_pad(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_gold = np.array([20, 50, 100])  # Lower bound for gold
    upper_gold = np.array([40, 255, 255])  # Upper bound for gold
    
    mask = cv2.inRange(hsv, lower_gold, upper_gold)
    
    # Show the mask to debug
    plt.imshow(mask, cmap='gray')
    plt.title("Gold Pad HSV Mask")
    plt.show()

    result_image = cv2.bitwise_and(image, image, mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pad_center = None
    for contour in contours:
        if cv2.contourArea(contour) > 200:  # Only consider large enough contours (pads)
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            cv2.circle(result_image, center, int(radius), (0, 255, 0), 2)  # Draw circle around pad
            cv2.circle(result_image, center, 5, (0, 0, 255), -1)  # Mark center with red dot
            pad_center = center
    
    if pad_center:
        print(f"Gold pad detected at: {pad_center}")
    else:
        print("No gold pad detected.")

    return result_image, pad_center

def detect_wire_and_tips(image, pad_center):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # Show the edges for debugging
    plt.imshow(edges, cmap='gray')
    plt.title("Canny Edge Detection")
    plt.show()

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=30, maxLineGap=5)

    wire_tip = None
    result_image = image.copy()

    if lines is not None:
        endpoints = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(result_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw the detected wire line in blue
            cv2.circle(result_image, (x1, y1), 5, (0, 0, 255), -1)  # Red dot on the start of the line
            cv2.circle(result_image, (x2, y2), 5, (0, 0, 255), -1)  # Red dot on the end of the line
            endpoints.extend([(x1, y1), (x2, y2)])

        # Calculate intersections between the lines
        intersections = []
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                x1, y1, x2, y2 = lines[i][0]
                x3, y3, x4, y4 = lines[j][0]

                # Calculate the intersection of lines (x1, y1)-(x2, y2) and (x3, y3)-(x4, y4)
                intersection = line_intersection((x1, y1), (x2, y2), (x3, y3), (x4, y4))

                if intersection:
                    intersections.append(intersection)
                    cv2.circle(result_image, intersection, 5, (0, 0, 255), -1)  # Draw red dot at the intersection

    return result_image

def line_intersection(p1, p2, p3, p4):
    """
    Find the intersection point of two lines (p1, p2) and (p3, p4).
    Each point is a tuple (x, y).
    Returns the intersection point (x, y) if the lines intersect, or None if they don't.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    # Denominator
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if denom == 0:
        return None  # Lines are parallel or coincident, no intersection

    # Numerators
    num1 = (x1 * y2 - y1 * x2)
    num2 = (x3 * y4 - y3 * x4)

    # Intersection point
    ix = (num1 * (x3 - x4) - (x1 - x2) * num2) / denom
    iy = (num1 * (y3 - y4) - (y1 - y2) * num2) / denom

    # Check if the intersection is within the line segments
    if min(x1, x2) <= ix <= max(x1, x2) and min(y1, y2) <= iy <= max(y1, y2) and min(x3, x4) <= ix <= max(x3, x4) and min(y3, y4) <= iy <= max(y3, y4):
        return (int(ix), int(iy))
    else:
        return None

def open_camera(device_index=0, width=320, height=240, fps=15):
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        print(f"Unable to open camera at index {device_index}")
    else:
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
                break
            elif key == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

    return captured_image

# Main execution
if __name__ == "__main__":
    captured_image = camera_feed()

    if captured_image is not None:
        gold_pad_image, pad_center = detect_gold_pad(captured_image)

        if pad_center:
            processed_image = detect_wire_and_tips(captured_image.copy(), pad_center)

            plt.figure(figsize=(10, 10))
            plt.imshow(cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB))
            plt.title("Detected Wire and Gold Pad with Intersections")
            plt.axis("off")
            plt.show()
        else:
            print("Gold pad not detected.")
    else:
        print("No image was captured.")
