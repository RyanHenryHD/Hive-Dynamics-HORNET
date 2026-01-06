from pymavlink import mavutil
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_1 = 1           # MAIN OUT 1
SERVO_2 = 2
PWM_NEUTRAL = 1500
PWM_MAX = 2000
STEP = 10           # Smaller = smoother
DELAY = 0.02        # 20 ms (~50 Hz)

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()
print("Connected")

def set_servo(servo, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo,
        pwm,
        0, 0, 0, 0, 0
    )

#SNAPPY MOVEMENT
print("Testing Snappy Movement")
# Set neutral first
print("Setting servos to NEUTRAL")
set_servo(SERVO_1, PWM_NEUTRAL)
set_servo(SERVO_2, PWM_NEUTRAL)
time.sleep(2)

# Move to MAX
print("Moving servos to MAX")
set_servo(SERVO_1, PWM_MAX)
set_servo(SERVO_2, PWM_MAX)
time.sleep(2)

# Return to NEUTRAL
print("Returning servos to NEUTRAL")
set_servo(SERVO_1, PWM_NEUTRAL)
set_servo(SERVO_2, PWM_NEUTRAL)
print("Snappy Movement Test Complete")

print("Testing Smooth Movement")
# Move from neutral to max
for pwm in range(PWM_NEUTRAL, PWM_MAX + 1, STEP):
    set_servo(SERVO_1, pwm)
    time.sleep(DELAY)
for pwm in range(PWM_NEUTRAL, PWM_MAX + 1, STEP):
    set_servo(SERVO_2, pwm)
    time.sleep(DELAY)

time.sleep(1)

# Move back to neutral
for pwm in range(PWM_MAX, PWM_NEUTRAL - 1, -STEP):
    set_servo(SERVO_1, pwm)
    time.sleep(DELAY)
for pwm in range(PWM_MAX, PWM_NEUTRAL - 1, -STEP):
    set_servo(SERVO_2, pwm)
    time.sleep(DELAY)

print("Smooth Movement Test Complete")