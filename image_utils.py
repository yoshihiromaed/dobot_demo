import cv2
import numpy as np

# detect_color_region
# This function detects a specific color (red or blue) region in an image and returns the centroid of that region.
def detect_color_region(img, color='red'):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if color == 'red':
        lower1 = np.array([0, 120, 70])
        upper1 = np.array([10, 255, 255])

        lower2 = np.array([170, 120, 70])
        upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = mask1 + mask2
    else: # Assuming blue color
        lower1 = np.array([85, 50, 50])
        upper1 = np.array([130, 255, 255])

        mask = cv2.inRange(hsv, lower1, upper1)

    red_region = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow("Region", red_region)
    cv2.erode(mask, None, mask, iterations=2)
    cv2.dilate(mask, None, mask, iterations=5)
    cv2.imshow("Mask", mask)

    moments = cv2.moments(mask)

    if moments["m00"] != 0:
        cX = int(moments["m10"] / moments["m00"])
        cY = int(moments["m01"] / moments["m00"])
    else:
        cX, cY = 0, 0

    print(f"centroid: ({cX}, {cY})")

    return cX, cY, mask
