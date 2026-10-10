import cv2
import serial
import time

# --- 1. SETUP SERIAL COMMUNICATION TO SENDER ESP32 ---
SERIAL_PORT = 'COM4'  # Update with your active port ('COMx' or '/dev/ttyUSB0')
BAUD_RATE = 115200

try:
    esp32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    time.sleep(2)  # Wait for serial bridge initialization
    print(f"Connected to Sender ESP32 on {SERIAL_PORT}")
except Exception as e:
    print(f"Serial port connection error: {e}")
    esp32 = None

# --- 2. VIDEO STREAM & ARUCO CONFIGURATION ---
STREAM_URL = "http://192.168.10.228:8080/video"  # Update with active IP Camera URL
cap = cv2.VideoCapture(STREAM_URL)

# Configured for 4x4 ArUco Grid (DICT_4X4_50) matching ID 0
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Target ArUco ID
TARGET_ID = 0

# Base pan and tilt initial positions (Degrees)
pan_angle = 90.0
tilt_angle = 90.0

# --- TUNED PARAMETERS TO PREVENT OVERSHOOTING & JITTER ---
Kp_pan = 0.015   # Low proportional gain stops motors from swinging past target
Kp_tilt = 0.015  # Low proportional gain stops motors from swinging past target
DEADBAND = 12    # Pixel tolerance window stops motor jitter at center

def send_servo_angles(p, t):
    """Formats and transmits P<pan>T<tilt> command string over serial to ESP32."""
    if esp32 and esp32.is_open:
        cmd = f"P{int(p)}T{int(t)}\n"
        esp32.write(cmd.encode('utf-8'))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Failed to capture frame from stream. Retrying...")
        continue

    # --- PORTRAIT ROTATION CORRECTION FOR UPRIGHT S24 ULTRA ---
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    height, width, _ = frame.shape
    frame_center_x = width // 2
    frame_center_y = height // 2

    # Draw central crosshair HUD overlay
    cv2.line(frame, (frame_center_x - 15, frame_center_y), (frame_center_x + 15, frame_center_y), (255, 0, 0), 2)
    cv2.line(frame, (frame_center_x, frame_center_y - 15), (frame_center_x, frame_center_y + 15), (255, 0, 0), 2)

    # --- 3. DETECT ARUCO MARKERS ---
    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None and TARGET_ID in ids.flatten():
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        idx = list(ids.flatten()).index(TARGET_ID)
        marker_corners = corners[idx][0]

        # Calculate geometric center (X, Y) of the target marker
        target_x = int(marker_corners[:, 0].mean())
        target_y = int(marker_corners[:, 1].mean())

        # Highlight target centroid
        cv2.circle(frame, (target_x, target_y), 6, (0, 255, 0), -1)

        # --- 4. ERROR VECTOR & DUAL-AXIS CORRECTION ---
        error_x = target_x - frame_center_x
        error_y = target_y - frame_center_y

        # Smooth positional adjustment toward center without overshooting
        if abs(error_x) > DEADBAND:
            pan_angle -= error_x * Kp_pan

        if abs(error_y) > DEADBAND:
            tilt_angle -= error_y * Kp_tilt

        # Constrain pan and tilt within physical 0 to 180 degree limits
        pan_angle = max(0, min(180, pan_angle))
        tilt_angle = max(0, min(180, tilt_angle))

        # Transmit updated coordinates to ESP32
        send_servo_angles(pan_angle, tilt_angle)

        cv2.putText(frame, f"LOCKED & CENTERED: ID {TARGET_ID}", (target_x + 10, target_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "TARGET LOST - SEARCHING", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Display HUD status overlay
    cv2.putText(frame, f"AeroBeam Gimbals | Pan (GPIO 18): {int(pan_angle)} deg | Tilt (GPIO 19): {int(tilt_angle)} deg",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    cv2.imshow("AeroBeam S24 Ultra Portrait Tracking HUD", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
if esp32:
    esp32.close()
cv2.destroyAllWindows()
