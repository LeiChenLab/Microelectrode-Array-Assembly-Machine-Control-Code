import cv2

def open_camera(device_index, width=320, height=240, fps=15):
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        print(f"Unable to open camera at index {device_index}")
    else:
        print(f"Camera opened successfully at index {device_index}")
        # Set resolution and frame rate
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def test_camera(camera_index):
    camera = open_camera(camera_index)
    if not camera.isOpened():
        print(f"Camera at index {camera_index} failed to open.")
        return

    print(f"Camera at index {camera_index} opened successfully. Press 'q' to exit.")

    while True:
        ret, frame = camera.read()
        if not ret or frame is None:
            print(f"Failed to capture image from camera at index {camera_index}.")
            break

        cv2.imshow(f'Camera {camera_index}', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

def detect_cameras(max_cameras=10):
    available_cameras = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras

if __name__ == "__main__":
    try:
        cameras = detect_cameras()
        if not cameras:
            print("No cameras found.")
        else:
            for index in cameras:
                test_camera(index)
    except KeyboardInterrupt:
        print("Program terminated.")
