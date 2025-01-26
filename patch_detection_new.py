import cv2
import numpy as np
import matplotlib.pyplot as plt

# Read original image
image = cv2.imread('img3.jpg')

# Sharpen the image
kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
sharpened = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)

# Define the regions to blur
regions_to_blur = [
    (0, 0, 520, 1200),
    (1378, 9, 471, 1187),
    (540, 1136, 1112, 100),
    (539,18,811,939)
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

# Canny Edge Detection
canny = cv2.Canny(image=img_gray, threshold1=100, threshold2=200)

# Find contours
contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter contours based on area ratio (assuming the smaller region is 1/5.29 of the larger)
big_area = 763634  # Approximate big area value
small_area_ratio = 1 / 5.29

# Draw contours for the small area
for cnt in contours:
    area = cv2.contourArea(cnt)
    if small_area_ratio * big_area * 0.9 < area < small_area_ratio * big_area * 1.1:  # Threshold to account for variation
        cv2.drawContours(image, [cnt], -1, (0, 0, 255), 2)  # Highlight in red

# Display results
plt.figure(figsize=(10, 5))
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Canny Edge Detection with Red Boundaries around Small Area')
plt.axis('off')
plt.show()
