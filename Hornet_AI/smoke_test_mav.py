# smoke_test_mav.py
from mavlink_interface import Pixhawk
import time

# adjust device path if your pixhawk is different
px = Pixhawk(device='/dev/serial0', baud=115200)
px.connect()

print("Attempting to read battery & location (non-blocking)...")
bat = px.get_battery(blocking=False)
print("Battery read:", bat)
pos = px.get_global_position(blocking=False)
print("Position read:", pos)

px.close()
print("Done.")
