from pymavlink import mavutil

# Connect to Cube via USB
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        print(f"Roll: {msg.roll:.2f}, Pitch: {msg.pitch:.2f}, Yaw: {msg.yaw:.2f}")