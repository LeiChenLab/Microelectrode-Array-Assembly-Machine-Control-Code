import cv2
import numpy as np
import matplotlib.pyplot as plt

# Read original image
image = cv2.imread('img3.jpg')

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

# Canny Edge Detection
canny = cv2.Canny(image=img_gray, threshold1=100, threshold2=200)

# Apply mask to Canny output
canny_masked = cv2.bitwise_and(canny, mask)

# Display results

plt.figure(figsize=(15, 5))

plt.subplot(1, 3, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Original Image')
plt.axis('off')

plt.subplot(1, 3, 2)
plt.imshow(cv2.cvtColor(sharpened, cv2.COLOR_BGR2RGB))
plt.title('Sharpened and Selectively Blurred')
plt.axis('off')

plt.subplot(1, 3, 3)
plt.imshow(canny_masked, cmap='gray')
plt.title('Canny Edge Detection (Blurred Regions Masked)')
plt.axis('off')

plt.tight_layout()
plt.show()

# Detect contours from canny_masked
contours, _ = cv2.findContours(canny_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Create an empty image to draw contours on
patch_image = np.copy(sharpened)

# Draw contours
cv2.drawContours(patch_image, contours, -1, (0, 255, 0), 2)

# Display results
plt.figure(figsize=(10, 5))

plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(sharpened, cv2.COLOR_BGR2RGB))
plt.title('Processed Image')
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(patch_image, cv2.COLOR_BGR2RGB))
plt.title(f'Detected Patches: {len(contours)}')
plt.axis('off')

plt.tight_layout()
plt.show()
