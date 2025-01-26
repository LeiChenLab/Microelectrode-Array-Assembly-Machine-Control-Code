import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_rectangles_with_preprocessing(image, target_width=70, target_height=100, height_tolerance=20, width_tolerance=18):
    if image is None or image.size == 0:
        raise ValueError("Input image is empty or None.")
    
    height, width = image.shape[:2]
    kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharpened = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)


    mask = np.ones(image.shape[:2], dtype=np.uint8) * 255
    

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

def open_camera(device_index, width=320, height=240, fps=15):
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
