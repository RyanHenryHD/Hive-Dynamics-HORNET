from pymavlink import mavutil

# Connect to Pixhawk via GPIO pins
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Connect to Pixhawk via USB
# master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

######################################################################################################
'''
1. Enable serial port on Pi4.
    Run: sudo raspi-config
        Interface Options → Serial Port
            Disable the login shell over serial but enable the serial port hardware.
                Save and reboot.

2. Check and Use the Correct Serial Device
    Run: ls -l /dev/serial0
    
3. Add User to dialout Group
    Run: sudo usermod -a -G dialout $USER
        Reboot

4. Install pymavlink 
    Run: pip install pymavlink

5. Run mavlink_test.py   

'''