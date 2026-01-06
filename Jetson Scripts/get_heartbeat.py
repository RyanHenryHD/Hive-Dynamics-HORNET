from pymavlink import mavutil

# Connect to Cube via USB
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system %u component %u" %
      (master.target_system, master.target_component))