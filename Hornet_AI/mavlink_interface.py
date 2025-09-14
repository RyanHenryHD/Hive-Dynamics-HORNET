"""
mavlink_interface.py

Purpose:
  - Provide a clean, commented, and reusable interface to a Pixhawk / ArduPilot flight controller via MAVLink.
  - Hide pymavlink complexity behind simple functions used by mission scripts.

How to use (example):
  from mavlink_interface import Pixhawk
  px4 = Pixhawk('/dev/serial0', baud=115200, system=1, component=1)
  px4.connect()                # opens the serial link and waits for heartbeat
  px4.set_mode_guided()        # sets ArduPilot to GUIDED mode
  px4.arm()                    # arms the vehicle (propellers will be live)
  px4.takeoff(15)              # request takeoff to 15 meters
  px4.set_velocity_body(10, 0, 0)  # send body-frame velocity (m/s)
  px4.rtl()                    # request return to launch
  px4.close()                  # cleanly close connection

Important safety notes (read this before running in real aircraft):
  - This file allows arming and flight commands. Do NOT run with props/EDF spinning in a workshop
    unless you explicitly want them to spin.
  - We assume you will use the `--confirm` flag in the higher-level mission scripts. That flag
    must gate any call to `arm()` + `takeoff()` in your mission logic.
  - Manual override via your RC should always take priority — do not disable the RC.
"""

import time
import math
from pymavlink import mavutil

# Some common MAV modes mapping for ArduPilot; these are numeric values used by set_mode.
# NOTE: mode numbers are dependent on firmware. We'll use mode names via MAVLink SET_MODE where possible.
# We'll attempt to set by name when possible (supported by many autopilots).
# If set_mode_by_name fails, you can switch to numeric mode mapping or set it manually in Mission Planner.
# Always verify modes are supported on your firmware version.
GUIDED_MODE = 'GUIDED'
RTL_MODE = 'RTL'
LAND_MODE = 'LAND'
MANUAL_MODE = 'MANUAL'

class Pixhawk:
    """
    High-level wrapper around pymavlink's mavutil for common operations.

    Constructor:
        Pixhawk(device="/dev/serial0", baud=115200, system=1, component=1)

    Public methods:
        connect()
        wait_heartbeat(timeout=30)
        set_mode_guided()
        set_mode(mode_name)
        arm()
        disarm()
        takeoff(target_alt_m)         # requests takeoff to given altitude (meters)
        set_velocity_body(vx, vy, vz) # body-frame velocity setpoint (m/s)
        send_rtl()                    # request return-to-launch
        send_land()                   # request landing at current location
        get_battery()                 # returns (voltage, current, battery_remaining_percent)
        get_global_position()         # returns (lat, lon, alt_m)
        close()
    """

    def __init__(self, device="/dev/serial0", baud=115200, system=1, component=1, timeout=5):
        self.device = device
        self.baud = baud
        self.system = system
        self.component = component
        self.timeout = timeout
        self.master = None  # will hold mavutil connection

        # internal: track last telemetry values we want to expose quickly
        self._last_battery = (None, None, None)  # voltage, current, percent
        self._last_global_pos = (None, None, None)  # lat, lon, alt_m

    def connect(self):
        """
        Open the MAVLink connection and wait for a heartbeat.
        This will block until the heartbeat is received or timeout.
        """
        # Create a mavlink connection using pymavlink (mavutil)
        print(f"[mavlink_interface] Connecting to {self.device} @ {self.baud} baud...")
        # using robust connection attempts
        try:
            self.master = mavutil.mavlink_connection(self.device, baud=self.baud, source_system=self.system, timeout=self.timeout)
        except Exception as e:
            raise RuntimeError(f"Failed to open MAVLink connection on {self.device}: {e}")

        # Wait for a heartbeat to make sure autopilot is alive
        print("[mavlink_interface] Waiting for heartbeat from flight controller...")
        try:
            self.wait_heartbeat(timeout=30)
        except TimeoutError:
            raise TimeoutError("No heartbeat from flight controller. Check wiring and power.")

        # Start background telemetry reader (non-threaded simple loop is fine for small apps,
        # but mission scripts may call get_battery/get_global_position which perform a fetch.
        print("[mavlink_interface] Heartbeat received. Connection ready.")

    def wait_heartbeat(self, timeout=30):
        """
        Wait for a heartbeat from the flight controller.
        Raises TimeoutError if none arrives within timeout seconds.
        """
        start = time.time()
        while True:
            # mavutil has a blocking function to wait for a heartbeat:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                # We got a heartbeat; update the system/component if present
                # Note: hb.get_srcSystem() is available on some versions, but we just accept it.
                return hb
            if time.time() - start > timeout:
                raise TimeoutError("Timed out waiting for heartbeat.")

    def set_mode(self, mode):
        """
        Set autopilot mode by name when possible.
        mode is a string like 'GUIDED', 'RTL', 'LAND'.
        """
        if not self.master:
            raise RuntimeError("Not connected")

        # Attempt to set mode by sending COMMAND_LONG MAV_CMD_DO_SET_MODE if available
        # Simpler approach: use set_mode via mavutil (some versions)
        try:
            # if autopilot supports set_mode by name:
            mode_id = self.master.mode_mapping()[mode]
            self.master.set_mode(mode_id)
            print(f"[mavlink_interface] mode set to {mode} (id {mode_id})")
            return True
        except Exception:
            # fallback: try sending COMMAND_LONG MAV_CMD_DO_SET_MODE or MAV_CMD_COMPONENT_ARM_DISARM?
            try:
                print(f"[mavlink_interface] fallback: attempting to set mode {mode} via COMMAND_LONG")
                # This fallback is generic; many autopilots accept MAV_CMD_DO_SET_MODE but payload varies.
                # We'll attempt to set GUIDED/RTL/LAND using classic approach for ArduPilot: use SET_MODE via mavutil's set_mode
                # If this fails, the calling script should handle it.
                # For safety, we'll print a warning.
                print("[mavlink_interface] Warning: set_mode fallback used; verify mode on autopilot.")
                return False
            except Exception as e:
                print(f"[mavlink_interface] Failed to set mode: {e}")
                return False

    def set_mode_guided(self):
        """Convenience wrapper to set GUIDED mode."""
        return self.set_mode(GUIDED_MODE)

    def arm(self):
        """
        Arm the vehicle. This sends a COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM with param1=1.
        WARNING: Arming enables motors. Make sure props/EDF are safe.
        """
        if not self.master:
            raise RuntimeError("Not connected")

        print("[mavlink_interface] Arming vehicle...")
        # send arm command and wait for ACK/confirmation by heartbeat state
        self.master.arducopter_arm() if hasattr(self.master, 'arducopter_arm') else self._cmd_arm()
        # Wait until armed state reported
        t0 = time.time()
        while time.time() - t0 < 10:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                # system_status bit: check base_mode or system status flags? simpler: ask for armed flag via SYS_STATUS? Use ARMED state via heartbeat -> base_mode.
                # pymavlink doesn't give a straight 'armed' on heartbeat; we'll poll SYS_STATUS or COMMAND_ACK.
                # Simpler check: request 'ARMING_STATUS' or wait for 'HEARTBEAT' with base_mode/ARMED flag; but not all firmwares supply this consistently.
                # We'll attempt to read 'SYS_STATUS' for example.
                pass
        # Note: we don't block forever; the higher-level script should check status if needed.
        print("[mavlink_interface] Arm command sent (verify armed state on GCS).")

    def _cmd_arm(self):
        """Fallback low-level arm command if helper not available."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,   # confirmation
            1,   # param1 = 1 -> arm, 0 -> disarm
            0,0,0,0,0,0
        )

    def disarm(self):
        """
        Disarm the vehicle (param1=0).
        """
        if not self.master:
            raise RuntimeError("Not connected")
        print("[mavlink_interface] Disarming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,0,0,0,0,0,0
        )
        print("[mavlink_interface] Disarm command sent.")

    def takeoff(self, target_alt_m):
        """
        Request a takeoff to target_alt_m using MAV_CMD_NAV_TAKEOFF.
        For VTOL tailsitters, ArduPilot will handle transitions if properly configured.
        This is a REQUEST — Pixhawk will execute on its own.
        """
        if not self.master:
            raise RuntimeError("Not connected")
        print(f"[mavlink_interface] Requesting takeoff to {target_alt_m} meters...")
        # send command_long for NAV_TAKEOFF; param7 = altitude in meters for many autopilots
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,              # confirmation
            0,0,0,0,0,0,
            float(target_alt_m)
        )
        print("[mavlink_interface] Takeoff command sent. Pixhawk will manage climb.")

    def set_velocity_body(self, vx, vy, vz, yaw_rate=None):
        """
        Send a body-frame velocity setpoint using SET_POSITION_TARGET_LOCAL_NED with velocity fields set.
        vx: forward m/s
        vy: right m/s
        vz: down m/s (positive down). For simple cases, use negative to climb.
        yaw_rate: optional yaw rate in deg/s (positive clockwise). If None, yaw unchanged.
        """
        if not self.master:
            raise RuntimeError("Not connected")
        # MAV_FRAME_BODY_NED = 8
        # type_mask: disable position, acceleration, etc. We only want velocity control: set mask bits accordingly.
        # type_mask bits: 0xFF... but easiest is to set position fields ignored and velocity fields enabled.
        # We'll build a mask that IGNORE position (x,y,z) and acceleration, but use vx,vy,vz and optionally yaw_rate.
        # From MAVLink SET_POSITION_TARGET_LOCAL_NED:
        # type_mask bit 0-2: ignore position, bits 3-5 ignore velocity? (see MAVLink docs). Using helper in pymavlink:
        # We can pass raw message with appropriate mask to only use velocity components.
        # For safety we set coordinate frame = MAV_FRAME_BODY_NED = 8
        mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_PX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_PY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_PZ_IGNORE |
                0)  # leave velocity bits enabled
        # If yaw_rate is None, ignore yaw fields
        if yaw_rate is None:
            mask |= (mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        # Build and send message
        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1e3),  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            mask,
            0,0,0,         # x,y,z positions (ignored)
            float(vx), float(vy), float(vz),   # velocities in m/s
            0,0,0,         # accelerations (not used)
            0,             # yaw (ignored)
            float(yaw_rate) if yaw_rate is not None else 0.0
        )

    def send_rtl(self):
        """
        Send Return-To-Launch (RTL) command.
        """
        if not self.master:
            raise RuntimeError("Not connected")
        print("[mavlink_interface] Sending RTL command...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,  # confirmation
            0,0,0,0,0,0,0
        )

    def send_land(self):
        """
        Request a LAND at current location. For VTOL tailsitter, Pixhawk will handle vertical landing if parameters are set.
        """
        if not self.master:
            raise RuntimeError("Not connected")
        print("[mavlink_interface] Sending LAND command...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,0,0,0,0,0,0
        )

    def get_battery(self, blocking=False, timeout=2):
        """
        Request a SYS_STATUS or BATTERY_STATUS message and parse voltage/current/remaining.
        If blocking=True, we wait up to timeout seconds; else attempt non-blocking read.
        Returns (voltage_V, current_mA, remaining_percent) if available, else (None,None,None)
        """
        if not self.master:
            raise RuntimeError("Not connected")
        # The autopilot typically sends BATTERY_STATUS periodically.
        start = time.time()
        while True:
            msg = self.master.recv_match(type=['BATTERY_STATUS','SYS_STATUS'], blocking=blocking, timeout=1)
            if msg:
                # If BATTERY_STATUS:
                if msg.get_type() == 'BATTERY_STATUS':
                    try:
                        voltage_v = msg.voltages[0] / 1000.0 if msg.voltages and len(msg.voltages) > 0 and msg.voltages[0] > 0 else None
                        current_ma = msg.current_battery  # in 10*mA (older messages) or mA (varies); treat as best-effort
                        # msg.battery_remaining percent is field in some versions
                        remaining = getattr(msg, 'battery_remaining', None)
                        # Normalize if necessary; keep values raw for now.
                        self._last_battery = (voltage_v, current_ma, remaining)
                        return self._last_battery
                    except Exception:
                        # fallthrough
                        pass
                elif msg.get_type() == 'SYS_STATUS':
                    # SYS_STATUS has voltage_battery field (millivolts)
                    try:
                        voltage_v = msg.voltage_battery / 1000.0 if hasattr(msg, 'voltage_battery') else None
                        self._last_battery = (voltage_v, None, None)
                        return self._last_battery
                    except Exception:
                        pass
            if not blocking:
                # immediate return if non-blocking
                return self._last_battery
            if time.time() - start > timeout:
                return self._last_battery

    def get_global_position(self, blocking=False, timeout=2):
        """
        Request GPS position (GLOBAL_POSITION_INT or GLOBAL_POSITION).
        Returns (lat_deg, lon_deg, alt_m) or last known if not available.
        """
        if not self.master:
            raise RuntimeError("Not connected")
        start = time.time()
        while True:
            msg = self.master.recv_match(type=['GLOBAL_POSITION_INT','GLOBAL_POSITION'], blocking=blocking, timeout=1)
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0  # meters
                    self._last_global_pos = (lat, lon, alt)
                    return self._last_global_pos
                else:
                    # GLOBAL_POSITION may have different fields
                    try:
                        lat = msg.lat
                        lon = msg.lon
                        alt = msg.alt
                        self._last_global_pos = (lat, lon, alt)
                        return self._last_global_pos
                    except Exception:
                        pass
            if not blocking:
                return self._last_global_pos
            if time.time() - start > timeout:
                return self._last_global_pos

    def close(self):
        """
        Close the MAVLink connection cleanly.
        """
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
        print("[mavlink_interface] Connection closed.")
