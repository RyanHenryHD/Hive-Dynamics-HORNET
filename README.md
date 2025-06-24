# Hive-Dynamics-HORNET
HORNET - AI-Powered Autonomous, VTOL, Reusable, Swarming Cruise Missile. 
________________________________________________________________________________________________________________________
MISSION PROMPT
-> Explaing the mission to llama, ranking targets, rules, etc

COMM PROTOCOL
-> Code on Raspberry Pi that pulls sensor input from Pixhawk Orange Cube+, Reads it, Sends back Pixhawk Readable Command
Communicates via MAVLINK
Example: 
MAVLINK READ: Pixhawk
  Targeting Lidar
  Thermal Cam (Runs through OpenCV - identifies targets)
  Visible Cam (Runs through OpenCV - identifies targets)
  Proximity Lidar
  AGL Lidar
  Airspeed Sensor
  GPS
  INS
  Terrain Cam (OpenCV or feature recognition library for GPS/Position estimation when jammed(paired with INS))
  Fuel Level: (61%) - estimated 22 minute remaining flight time

  RASPBERRY PI CODE FLOW:
  [Raspberry Pi] 
  ➡️ [Read MAVLink from Pixhawk] 
  ➡️ [Identify Sensor Messages: GPS, INS, etc.] 
  ➡️ [Trigger CV Processing: Thermal/Visual/AGL] 
  ➡️ [Decide Action: New Waypoint/loiter/avoid] 
  ➡️ [Send Command Back To Pixhawk Via MAVLink]
  
  CREATE LLAMA PROMPT
  "
  GPS: (Jammed)
  INS/T-CAM: (78.922742, 37.375932, 480m) 78% confidence
  Speed: 612 kph
  Airspeed: 597 kph
  Targets: (Tank1: 80.262, 38.278245, 0m) 97% confidence, (Tank2: 81.7662, 38.598245, 0m) 86% confidence, (Attack Helicopter: 76.26462, 35.129245, 325m) 91% confidence, (Drone: 80.262, 40.278245, 386m) 78% confidence, (Assualt Ship: 83.262, 35.364531, -7m) 97% confidence
  Obstacles: (Friendly_Hornet_1: 39.66482, 81.85783) 82% confidence, (Friendly_Hornet_2: 83.1127, 35.35735) 75% confidence, (Unknown: 42.66482, 81.78369)
  UPDATE ROLE/OBJECTIVE
  "

  LLAMA RESPONSE TO PIXHAWK
  "
  MISSION UPDATE: New Target: Assualt Ship
  UPDATE WAYPOINT: (83.262, 35.364531) -using pathfinder library/mpc-
  AVOID OBSTACLE: (Friendly_Hornet_2: 83.1127, 35.35735) -using pathfinder library/mpc-
  REASON: Assualt Ship marks highest priority given range and fuel capacity. Meneurvering around "Friendly_Hornet_2".
  TIME: 12:39:04
  "

SWARM LOGIC
-> (covered by Llama?)
