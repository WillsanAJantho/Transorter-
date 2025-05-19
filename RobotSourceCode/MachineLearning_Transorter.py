import cv2
import numpy as np
from scipy.spatial.distance import euclidean
from imutils import perspective
from imutils import contours
import imutils
import rectpack
import matplotlib.pyplot as plt
import time
import serial
import time
import math

# serialInst = serial.Serial()
# serialInst.baudrate = 9600
# serialInst.port = 'COM4'
# serialInst.open()


detected_rectangles = []
container_width = 15
container_height = 25
skip_first_object = True
offset_x = -13
offset_y = -20.5

# Function to round to the nearest 0.5
def round_to_nearest_half(x):
    return round(x * 2) / 2

# Initialize webcam
cap = cv2.VideoCapture(1)  # Replace 0 with your camera ID or video file path

# Check if the camera is opened
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# Define known dimensions of the reference object (e.g., 5 cm)
ref_object_width_cm = 5

# Store details of detected objects and count
printed_objects = set()
printed_count = 0
max_objects_to_print = 11
max_rectangles = 1
detected_rectangles = []
# Add a flag to track if the first object has been detected
first_object_detected = False

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

            # Convert center coordinates to cm with bottom-center as origin
            center_x_cm = (center_x_px - frame.shape[1] / 2) / pixel_per_cm  # X relative to center
            center_y_cm = (frame.shape[0] - center_y_px) / pixel_per_cm  # Y relative to bottom

            # Draw center dot
            cv2.circle(frame, (center_x_px, center_y_px), 5, (0, 0, 255), -1)

            # Measure dimensions
            width = euclidean(tl, tr) / pixel_per_cm
            height = euclidean(tr, br) / pixel_per_cm

            # Add margin of 0.5 cm to each dimension
            width += 0.5
            height += 0.5

            # Create a unique identifier for each object
            object_id = (round_to_nearest_half(width), round_to_nearest_half(height),
                         round_to_nearest_half(center_x_cm), round_to_nearest_half(center_y_cm))

            # Skip the first detected object
            if not first_object_detected:
                first_object_detected = True
                continue

            # Print details for the first 10 objects only
            if object_id not in printed_objects and printed_count < max_objects_to_print:
                printed_objects.add(object_id)
                printed_count += 1
                print(f"Object {printed_count}: Width={round_to_nearest_half(width)}cm, "
                      f"Height={round_to_nearest_half(height)}cm, "
                      f"Center=({round_to_nearest_half(center_x_cm)}cm, {round_to_nearest_half(center_y_cm)}cm)")

                # Add detected rectangle to the list (skip the first object)
                detected_rectangles.append((width, height, center_x_cm, center_y_cm, printed_count))
                time.sleep(1)

            # Display dimensions and coordinates on the frame
            mid_horizontal = (int((tl[0] + tr[0]) / 2), int((tl[1] + tr[1]) / 2))
            mid_vertical = (int((tr[0] + br[0]) / 2), int((tr[1] + br[1]) / 2))
            cv2.putText(frame, f"{round_to_nearest_half(width)}cm", (mid_horizontal[0] - 15, mid_horizontal[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame, f"{round_to_nearest_half(height)}cm", (mid_vertical[0] + 10, mid_vertical[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame, f"({round_to_nearest_half(center_x_cm)}, "
                               f"{round_to_nearest_half(center_y_cm)})cm", 
                        (center_x_px - 50, center_y_px + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 25, 255), 2)

    # Show the frame
    cv2.imshow("Live Measurement", frame)
    
    # Break on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if len(detected_rectangles) >= max_rectangles:
        time.sleep(3)
        break


# Perform hyper-packing if sufficient rectangles are detected
if len(detected_rectangles) >= max_rectangles:
    print("\nPerforming hyper-packing of detected rectangles:")

    # Initialize the packer and disable rotation
    packer = rectpack.newPacker(rotation=False)

    # Define the margin size (gap between rectangles)
    margin = 0.2  # Adjust as needed (in cm)

    # Add all detected rectangles to the packer with margins
    for rect in detected_rectangles:
        adjusted_width = rect[0] + margin
        adjusted_height = rect[1] + margin
        rect_id = rect[4]
        # print(f" rect id = {rect_id}")
        packer.add_rect(adjusted_width, adjusted_height, rect_id)  # Add rectangles with the margin included

    # Add the container (bin) to the packer
    packer.add_bin(container_width, container_height)

    # Perform the packing process
    packer.pack()

    # Retrieve the packed rectangles
    packed_bins = packer.rect_list()

    # Map original rectangles to packed results
    packed_rectangles = []
    for rect_data in packed_bins:
        b, x, y, w, h, rid = rect_data

        # Subtract the margin to restore the original dimensions
        packed_rectangles.append({
            "id": rid,
            "x": x + margin / 2,  # Adjust x-coordinate for margin
            "y": y + margin / 2,  # Adjust y-coordinate for margin
            "width": round_to_nearest_half(w - margin),
            "height": round_to_nearest_half(h - margin),
        })

    # Match detected objects with packed rectangles
    object_to_packed_mapping = []
    center_points_mapping = []  # To store the final mapping of center points

    for i, obj in enumerate(detected_rectangles):
        obj_width, obj_height = round_to_nearest_half(obj[0]), round_to_nearest_half(obj[1])
        obj_id = obj[4]

        # Iterate through packed rectangles and map based on size
        for j, packed_rect in enumerate(packed_rectangles):

            packed_width = packed_rect["width"]
            packed_height = packed_rect["height"]
            packed_id = packed_rect["id"]

            # Match dimensions (allowing for rounding differences)
            if obj_id == packed_id:
                packed_center_x = packed_rect["x"] + packed_width / 2
                packed_center_y = packed_rect["y"] + packed_height / 2

                # Rectangle position and size (adjusted for the new origin)
                x = packed_rect["y"] - container_height / 2  # Adjust x for top-center origin
                y = -packed_rect["x"]  # Flip y for top-center origin
                w = packed_rect["height"]
                h = packed_rect["width"]

                # Rectangle center (adjusted for the new origin)
                center_x = x + w / 2
                center_y = y - h / 2

                # Update mapping with the correct packed center
                object_to_packed_mapping.append({
                    "object": i + 1,
                    "original_width": obj_width,
                    "original_height": obj_height,
                    "original_center": f"({round_to_nearest_half(obj[2]):.1f}, {round_to_nearest_half(obj[3]):.1f})",
                    "packed_width": packed_width,
                    "packed_height": packed_height,
                    "packed_center": f"({center_x:.1f}, {center_y:.1f})",
                    "packed_id": packed_id
                })

                # Store the center points of both detected and packed rectangles
                center_points_mapping.append([round_to_nearest_half(obj[2]), round_to_nearest_half(obj[3])])
                center_points_mapping.append([round(center_x, 1), round(center_y, 1)])
                # Use the packed rectangle's center from output
                packed_rect["used"] = True  # Mark as used
                break

    # Output the mapping
    # print("\nObject-to-Packed Mapping:")
    # for mapping in object_to_packed_mapping:
    #     print(f"Object {mapping['object']}: "
    #           f"Width={mapping['original_width']}cm, Height={mapping['original_height']}cm, "
    #           f"Center={mapping['original_center']}, "
    #           f"Packed Size=({mapping['packed_width']}cm, {mapping['packed_height']}cm), "
    #           f"Packed Center={mapping['packed_center']}, "
    #           f"ID {mapping['packed_id']}: ")



    # Plot the packing solution with a horizontal container and top-center origin
    fig, ax = plt.subplots(figsize=(10, 6))  # Adjusted width for horizontal layout

    # Display the container as a rectangle (horizontal orientation)
    ax.add_patch(plt.Rectangle((-container_height / 2, 0), container_height, container_width, 
                                fill=None, edgecolor='black', linewidth=2))
    
    # Add each packed rectangle to the plot with corresponding labels
    for idx, rect in enumerate(packed_rectangles):
        # Rectangle position and size (adjusted for the new origin)
        x = rect["y"] - container_height / 2  # Adjust x for top-center origin
        y = -rect["x"]  # Flip y for top-center origin
        w = rect["height"]
        h = rect["width"]
        rectangle_id = rect["id"]

        # Rectangle center (adjusted for the new origin)
        center_x = x + w / 2
        center_y = y - h / 2

        # Draw the rectangle
        ax.add_patch(plt.Rectangle((x, y), w, -h, fill=True, edgecolor='blue', facecolor='skyblue', linewidth=2))

        # Add the size label inside the rectangle
        ax.text(
            center_x,
            center_y,  # Center of the rectangle
            f"S:{w}x{h}",
            fontsize=8,
            color="red",
            ha="center",
            va="center"
        )

        # Add the coordinate label slightly below the rectangle
        ax.text(
            center_x,
            center_y - 0.5,  # Slightly below the rectangle
            f"Cor:({round(center_x, 1)}, {round(center_y, 1)})",
            fontsize=8,
            color="black",
            ha="center",
            va="top"
        )

        # # Log the details to the terminal
        # print(f"Packed rectangle {idx + 1}: Center=({center_x:.1f}, {center_y:.1f}), Size=({w}cm, {h}cm), ID = ({rectangle_id})")
    
    # Output the center points mapping
    print("\nCenter Points Mapping:")
    # for idx, point in enumerate(center_points_mapping, start=1):  # start=1 to start the count from 1
        # print(f"Rectangle {idx}: {point}")
    
    print(center_points_mapping)

    for value in center_points_mapping:
        input("Press Enter to continue...")
        print(value)
        data = f"{value[0]},{value[1]}"
        # serialInst.write(data.encode())
        print(f"Sent: {data}")



    # Set axis labels, limits, and title
    ax.set_xlim(-container_height / 2, container_height / 2)
    ax.set_ylim(-container_width, 0)
    ax.set_title("Horizontal Packing Result (Top-Center Origin)")
    ax.set_xlabel("Length (cm)")
    ax.set_ylabel("Width (cm)")

    # Maintain equal aspect ratio for proper visualization
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()

# Save the processed frame when 's' is pressed
if cv2.waitKey(1) & 0xFF == ord('s'):
    cv2.imwrite("detected_objects.jpg", frame)
    print("Processed image saved as 'detected_objects.jpg'")

# Release resources
cap.release()
cv2.destroyAllWindows()