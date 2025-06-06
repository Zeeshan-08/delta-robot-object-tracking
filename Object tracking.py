import cv2
import numpy as np
import serial
import time

# === Initialize Serial Communication ===
ser = serial.Serial('COM3', 38400)  # Replace with your actual port
time.sleep(2)

# === Video Capture ===
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

if not cap.isOpened():
    print("Camera couldn't access")
    exit()

# === Object Detection Parameters (adjust color range for your object) ===
lower_color = np.array([30, 100, 100])  # HSV lower bound (e.g., green)
upper_color = np.array([90, 255, 255])  # HSV upper bound

# === Stability Detection ===
prev_center = None
stable_counter = 0
STABLE_THRESHOLD = 10  # Number of frames object must remain still

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV and create mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Detect contours
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None

    if len(cnts) > 0:
        # Find largest object
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:  # Filter out small noise
            center = (int(x), int(y))
            cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
            cv2.putText(frame, f"Target: ({int(x)}, {int(y)})", (int(x)+10, int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            # Check if object is stable
            if prev_center is not None and np.linalg.norm(np.array(center) - np.array(prev_center)) < 5:
                stable_counter += 1
            else:
                stable_counter = 0

            prev_center = center

            # Send data after object is stable
            if stable_counter >= STABLE_THRESHOLD:
                z = -450  # Fixed height for picking
                serial_data = f"<{x:.2f},{y:.2f},{z:.2f}>"
                ser.write(serial_data.encode())
                print(f"Sent: {serial_data}")
                stable_counter = 0  # Reset
                time.sleep(2)

    else:
        cv2.putText(frame, "No Object Detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Show video
    cv2.imshow("Object Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === Cleanup ===
cap.release()
cv2.destroyAllWindows()
ser.close()
