# main.py
import time
from pymavlink import mavutil
from configure import (
    CRUISE_SPEED_MS,
    INTERCEPT_SPEED_MS,
    SEARCH_ALTITUDE_M,
    MIN_ALTITUDE_M,
    MAX_ALTITUDE_M,
    LOST_TARGET_TIMEOUT,
)
from mavlink_interface import connect_pixhawk, arm_and_takeoff, set_velocity_body
from camera_module import Camera, detect_targets


def mode_hover_test(px):
    """Simple hover at SEARCH_ALTITUDE_M for system check."""
    print("[MODE] Hover Test")
    arm_and_takeoff(px, SEARCH_ALTITUDE_M)
    print("[INFO] Holding hover. Ctrl+C to exit.")
    while True:
        time.sleep(1)


def mode_basic(px):
    """Basic flight at SEARCH_ALTITUDE_M with cruise speed."""
    print("[MODE] Basic Flight")
    arm_and_takeoff(px, SEARCH_ALTITUDE_M)
    print(f"[INFO] Flying forward at {CRUISE_SPEED_MS} m/s")
    while True:
        set_velocity_body(px, CRUISE_SPEED_MS, 0, 0)
        time.sleep(1)


def mode_targeting(px):
    """Search for targets with camera; intercept when found."""
    print("[MODE] Targeting")
    arm_and_takeoff(px, SEARCH_ALTITUDE_M)

    cam = Camera()
    last_seen_time = None

    while True:
        frame = cam.get_frame()
        detections = detect_targets(frame)

        if detections:
            # Found target -> intercept
            print("[INFO] Target detected! Intercepting at full throttle.")
            set_velocity_body(px, INTERCEPT_SPEED_MS, 0, 0)
            last_seen_time = time.time()

        else:
            # Lost or no target
            now = time.time()
            if last_seen_time and (now - last_seen_time) <= LOST_TARGET_TIMEOUT:
                # Give chance to reacquire while flying level
                print("[INFO] Lost visual, holding cruise speed for possible reacquire...")
                set_velocity_body(px, CRUISE_SPEED_MS, 0, 0)
            else:
                # Resume normal search
                print("[INFO] Scanning... cruising forward.")
                set_velocity_body(px, CRUISE_SPEED_MS, 0, 0)

        # Safety altitude guard (dummy check – real enforcement handled in PX4/ArduPilot params)
        if SEARCH_ALTITUDE_M < MIN_ALTITUDE_M or SEARCH_ALTITUDE_M > MAX_ALTITUDE_M:
            print("[WARN] Search altitude out of safety bounds! Adjust configure.py")
            break

        time.sleep(0.5)


def main():
    print("[SYSTEM] Connecting to Pixhawk...")
    px = connect_pixhawk()

    print("[SYSTEM] Select mode:")
    print("1. Hover Test")
    print("2. Basic Flight")
    print("3. Targeting")
    mode = input("Enter choice: ")

    if mode == "1":
        mode_hover_test(px)
    elif mode == "2":
        mode_basic(px)
    elif mode == "3":
        mode_targeting(px)
    else:
        print("[ERROR] Invalid mode selected.")


if __name__ == "__main__":
    main()
