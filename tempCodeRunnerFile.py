import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_rectangles_with_preprocessing(image_path, target_width=70,target_height=100, height_tolerance=20, width_tolerance=18):
    # Read original image
    image = cv2.imread(image_path)
    height, width = image.shape[:2]
    # Sharpen the image
    kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharpened = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)

    # Define the regions to blur
    regions_to_blur = [
        (0, 0, 520, 1200),
        (1378, 9, 471, 1187),
        (540, 1136, 1112, 100),
        (539, 18, 811, 939)
    ]

    # Create a mask for blurred regions
    mask = np.ones(image.shape[:2], dtype=np.uint8) * 255

    # Apply Gaussian blur to specific regions and update mask
    for (x, y, w, h) in regions_to_blur:
        region = sharpened[y:y+h, x:x+w]
        blurred_region = cv2.GaussianBlur(region, (25, 25), 0)
        sharpened[y:y+h, x:x+w] = blurred_region
        mask[y:y+h, x:x+w] = 0

    # Convert to grayscale
    img_gray = cv2.cvtColor(sharpened, cv2.COLOR_BGR2GRAY)

    # Apply the mask to the grayscale image
    img_gray_masked = cv2.bitwise_and(img_gray, mask)
    
    # Apply histogram equalization
    equalized = cv2.equalizeHist(img_gray_masked)
    # Threshold the masked grayscale image
    _, binary = cv2.threshold(img_gray_masked, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
     # Apply morphological operations
    kernel = np.ones((3, 3), np.uint8)
    img_gray_masked = cv2.morphologyEx(img_gray_masked, cv2.MORPH_CLOSE, kernel, iterations=2)
    img_gray_masked = cv2.morphologyEx(img_gray_masked, cv2.MORPH_OPEN, kernel,iterations=1)
    
    edges = cv2.Canny(equalized, 10, 200)
    #improve contrast with histogram equilaization
    clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(img_gray_masked)
    thresh=cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)    
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create an empty image to draw contours on
    patch_image = np.copy(sharpened)

    # Filter and draw rectangles
    rectangles = []
    centers = []
    min_area=230
    for contour in contours:
        epsilon = 0.0005 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        x, y, w, h = cv2.boundingRect(contour)
        if y > height * 0.75 and y<height*0.90:
            if len(approx == 4) and (abs(w - target_width) <= width_tolerance) or (abs(h-target_height)<=height_tolerance) or cv2.contourArea(contour) >= min_area :
                rectangles.append((x, y, w, h))
                cv2.rectangle(patch_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                center_x = x + w // 2
                center_y = y + h // 2
                centers.append((center_x, center_y))
                # Draw the center point on the image
                cv2.circle(patch_image, (center_x, center_y), 5, (0, 0, 255), -1)
    return sharpened, patch_image, len(rectangles), centers

# Usage
image_path = 'img3.jpg'
processed_image, patch_image, num_rectangles, num_centers = detect_rectangles_with_preprocessing(image_path)

# Display results
plt.figure(figsize=(15, 5))

plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB))
plt.title('Processed Image')
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(patch_image, cv2.COLOR_BGR2RGB))
plt.title(f'Detected Rectangles: {num_rectangles}')
plt.axis('off')

plt.tight_layout()
plt.show()