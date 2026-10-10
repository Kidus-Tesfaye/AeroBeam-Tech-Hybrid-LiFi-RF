import cv2
import math
import serial
import socket
import sys
import time

# --- 1. SETUP SERIAL COMMUNICATION TO SENDER ESP32 ---
SERIAL_PORT = 'COM3'  # Update with your active port ('COMx' or '/dev/ttyUSB0')
BAUD_RATE = 115200

# pip's "serial" package is an unrelated library that also imports as `serial`.
# Only pySerial has serial.Serial, and without it the servos never get a command.
if not hasattr(serial, "Serial"):
    sys.exit("The installed 'serial' package is not pySerial, so the servos can't be driven.\n"
             "Fix the venv:  pip uninstall -y serial  then  pip install -r requirements.txt")

from serial.tools import list_ports

# No servo link means no tracking, so stop here instead of running a HUD that only looks healthy
try:
    esp32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
except serial.SerialException as e:
    ports = "\n".join(f"  {p.device}: {p.description}" for p in list_ports.comports())
    sys.exit(f"Can't open {SERIAL_PORT}: {e}\n"
             f"Serial ports on this PC:\n{ports or '  (none - is the sender ESP32 plugged in?)'}\n"
             "Set SERIAL_PORT to the sender's port (an ESP32 shows up as CP210x or CH340),\n"
             "and close any serial monitor that has it open.")
time.sleep(2)  # Opening the port can reboot the ESP32; let it finish booting
print(f"Connected to Sender ESP32 on {SERIAL_PORT}")
sender_rx = b""  # Partial line from the sender, waiting for its newline

# --- 2. UDP LINK COMMANDS TO RECEIVER ESP32 ---
# The receiver has no beam sensor: this script decides from the camera whether the
# laser is on the panel, and tells the receiver which audio link to play.
RECEIVER_IP = "192.168.10.50"  # Update with the IP shown on the receiver's OLED
RECEIVER_PORT = 4210           # Must match HOST_CMD_PORT in firmware/src/receiver/main.cpp

link_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- 3. VIDEO STREAM & ARUCO CONFIGURATION ---
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
Kp_pan = 0.007   # Low proportional gain stops motors from swinging past target
Kp_tilt = 0.008  # Low proportional gain stops motors from swinging past target
DEADBAND = 12    # Pixel tolerance window stops motor jitter at center

# --- LIFI / RF LINK RULE (replaces the solar-panel voltage sensing) ---
# LiFi whenever the target marker is anywhere in the frame, centred or not.
# RF only once the camera has lost the marker completely for LOST_GRACE_S. The grace
# period rides out single frames where detection blinks (motion blur, glare), so the
# receiver doesn't chop between links. Set it to 0 to fall back on the first missed frame.
LOST_GRACE_S = 0.3
HEARTBEAT_S = 0.1      # Resend the mode this often; the receiver fails over after 1 s of silence

link_mode = 'R'        # 'L' = LiFi (laser), 'R' = RF (ESP-NOW). Start on RF until the target is seen.
last_seen = None       # time.monotonic() of the last frame that contained the target
last_link_send = 0.0


def send_servo_angles(p, t):
    """Formats and transmits P<pan>T<tilt> command string over serial to ESP32.
    The firmware takes angles in tenths of a degree (P900T900 = 90.0 deg each)."""
    cmd = f"P{round(p * 10)}T{round(t * 10)}\n"
    esp32.write(cmd.encode('utf-8'))


def print_sender_output():
    """Echoes whatever the sender ESP32 prints (boot banner, Wi-Fi status, errors) to this console.
    A boot banner that keeps reappearing means the board is rebooting, often from servo power draw."""
    global sender_rx
    if esp32.in_waiting:
        sender_rx += esp32.read(esp32.in_waiting)
        *lines, sender_rx = sender_rx.split(b"\n")
        for line in lines:
            # Boot noise isn't valid text: show odd bytes as \xNN so any console can print them
            text = line.decode('ascii', errors='backslashreplace').strip()
            if text:
                print(f"[sender] {text}")


def send_link_mode(mode, offset):
    """Sends 'L<offset>' or 'R<offset>' to the receiver over UDP (offset -1 = target lost)."""
    try:
        link_sock.sendto(f"{mode}{offset}".encode('utf-8'), (RECEIVER_IP, RECEIVER_PORT))
    except OSError as e:
        print(f"Link command not sent: {e}")


def next_link_mode(last_seen, now):
    """'L' if the target was in view within the last LOST_GRACE_S seconds, otherwise 'R'."""
    if last_seen is not None and now - last_seen <= LOST_GRACE_S:
        return 'L'
    return 'R'


while cap.isOpened():
    print_sender_output()

    ret, frame = cap.read()
    now = time.monotonic()
    if not ret or frame is None:
        # No link command goes out either, so the receiver fails over to RF after 1 s
        print("Failed to capture frame from stream. Retrying...")
        continue

    # --- PORTRAIT ROTATION CORRECTION FOR UPRIGHT S24 ULTRA ---
    # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    height, width, _ = frame.shape
    frame_center_x = width // 2
    frame_center_y = height // 2

    # Draw central crosshair HUD overlay
    cv2.line(frame, (frame_center_x - 15, frame_center_y), (frame_center_x + 15, frame_center_y), (255, 0, 0), 2)
    cv2.line(frame, (frame_center_x, frame_center_y - 15), (frame_center_x, frame_center_y + 15), (255, 0, 0), 2)

    # --- 4. DETECT ARUCO MARKERS ---
    corners, ids, rejected = detector.detectMarkers(frame)
    offset = None

    if ids is not None and TARGET_ID in ids.flatten():
        last_seen = now  # In view anywhere in the frame counts: the link goes LiFi
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        idx = list(ids.flatten()).index(TARGET_ID)
        marker_corners = corners[idx][0]

        # Calculate geometric center (X, Y) of the target marker
        target_x = int(marker_corners[:, 0].mean())
        target_y = int(marker_corners[:, 1].mean())

        # Highlight target centroid
        cv2.circle(frame, (target_x, target_y), 6, (0, 255, 0), -1)

        # --- 5. ERROR VECTOR & DUAL-AXIS CORRECTION ---
        error_x = target_x - frame_center_x
        error_y = target_y - frame_center_y
        offset = int(math.hypot(error_x, error_y))

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

        centered = abs(error_x) <= DEADBAND and abs(error_y) <= DEADBAND
        cv2.putText(frame, f"TARGET ID {TARGET_ID}{' - CENTERED' if centered else ''}",
                    (target_x + 10, target_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "TARGET LOST - SEARCHING", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # --- 6. LIFI / RF LINK DECISION -> RECEIVER ESP32 ---
    mode = next_link_mode(last_seen, now)
    if mode != link_mode or now - last_link_send >= HEARTBEAT_S:
        if mode != link_mode:
            print(f"Link -> {'LiFi (laser)' if mode == 'L' else 'RF (ESP-NOW)'} (offset {offset} px)")
        link_mode = mode
        send_link_mode(link_mode, -1 if offset is None else offset)
        last_link_send = now

    # Link status: green while on LiFi, red while on RF
    link_color = (0, 255, 0) if link_mode == 'L' else (0, 0, 255)
    link_text = "LiFi (laser)" if link_mode == 'L' else "RF (ESP-NOW)"
    offset_text = "--" if offset is None else f"{offset} px"
    cv2.putText(frame, f"Link: {link_text} | Offset: {offset_text}", (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, link_color, 2)

    # Display HUD status overlay
    cv2.putText(frame, f"AeroBeam Gimbals | Pan (GPIO 18): {int(pan_angle)} deg | Tilt (GPIO 19): {int(tilt_angle)} deg",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    cv2.imshow("AeroBeam S24 Ultra Portrait Tracking HUD", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Tracking stops here, so tell the receiver to fall back to RF right away
send_link_mode('R', -1)
link_sock.close()
cap.release()
esp32.close()
cv2.destroyAllWindows()
