import cv2

def list_connected_cameras(max_tested=10):
    available_cameras = []
    for i in range(max_tested):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera found at index {i}")
            available_cameras.append(i)
            cap.release()
        else:
            cap.release()
    return available_cameras

if __name__ == "__main__":
    print("Listing all available cameras:")
    cameras = list_connected_cameras()
    print(f"Available cameras: {cameras}")
