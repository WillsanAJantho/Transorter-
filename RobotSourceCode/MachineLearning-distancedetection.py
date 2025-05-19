import cv2
import numpy as np
from scipy.spatial.distance import euclidean
from imutils import perspective
from imutils import contours
import imutils

# Initialize webcam
cap = cv2.VideoCapture(0)  # Replace 0 with your camera ID or video file path

# Check if the camera is opened
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# Define known dimensions of the reference object (e.g., 5 cm)
ref_object_width_cm = 5

# Store details of detected objects and count
printed_objects = set()
printed_count = 0
max_objects_to_print = 6

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame from the camera.")
        break

    # Rotate the frame 180 degrees
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Preprocessing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)
    edged = cv2.Canny(blur, 50, 100)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    # Find contours
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # Sort contours from left to right
    if cnts:
        (cnts, _) = contours.sort_contours(cnts)

    # Filter small contours
    cnts = [x for x in cnts if cv2.contourArea(x) > 100]

    # Compute pixel-per-centimeter scale using the reference object
    if cnts:
        ref_object = cnts[0]
        box = cv2.minAreaRect(ref_object)
        box = cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        (tl, tr, _, _) = box
        pixel_dist = euclidean(tl, tr)
        pixel_per_cm = pixel_dist / ref_object_width_cm

        # Draw contours and dimensions
        for i, cnt in enumerate(cnts):
            box = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            (tl, tr, br, bl) = box
            cv2.drawContours(frame, [box.astype("int")], -1, (0, 255, 0), 2)

            # Calculate center of the bounding box in pixels
            center_x_px = int((tl[0] + tr[0] + br[0] + bl[0]) / 4)
            center_y_px = int((tl[1] + tr[1] + br[1] + bl[1]) / 4)

            # Convert center coordinates to cm
            center_x_cm = (center_x_px - frame.shape[1] / 2) / pixel_per_cm
            center_y_cm = center_y_px / pixel_per_cm

            # Draw center dot
            cv2.circle(frame, (center_x_px, center_y_px), 5, (0, 0, 255), -1)

            # Measure dimensions
            width = euclidean(tl, tr) / pixel_per_cm
            height = euclidean(tr, br) / pixel_per_cm

            # Create a unique identifier for each object
            object_id = (round(width, 1), round(height, 1), round(center_x_cm, 1), round(center_y_cm, 1))

            # Print details for the first 6 objects only
            if object_id not in printed_objects and printed_count < max_objects_to_print:
                printed_objects.add(object_id)
                printed_count += 1
                print(f"Object {printed_count}: Width={width:.1f}cm, Height={height:.1f}cm, "
                      f"Center=({center_x_cm:.1f}cm, {center_y_cm:.1f}cm)")

            # Display dimensions and coordinates on the frame
            mid_horizontal = (int((tl[0] + tr[0]) / 2), int((tl[1] + tr[1]) / 2))
            mid_vertical = (int((tr[0] + br[0]) / 2), int((tr[1] + br[1]) / 2))
            cv2.putText(frame, f"{width:.1f}cm", (mid_horizontal[0] - 15, mid_horizontal[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame, f"{height:.1f}cm", (mid_vertical[0] + 10, mid_vertical[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame, f"({center_x_cm:.1f}, {center_y_cm:.1f})cm", 
                        (center_x_px - 50, center_y_px + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 25, 255), 2)

    # Add coordinate labels at the corners of the frame
    height, width, _ = frame.shape
    top_center = (width // 2, 20)
    bottom_center = (width // 2, height - 10)
    top_left = (20, 20)
    top_right = (width - 100, 20)
    cv2.putText(frame, "(0, 0)cm", top_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"({-(width // 2) / pixel_per_cm:.1f}, 0)cm", top_left, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"({(width // 2) / pixel_per_cm:.1f}, 0)cm", top_right, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, f"(0, {height / pixel_per_cm:.1f})cm", bottom_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Show the frame
    cv2.imshow("Live Measurement", frame)

    # Break on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
