Spray Control: Python → Arduino → Servo (Serial Protocol)

This repo/workspace implements a simple, reliable pipeline to trigger a spray mechanism:

Robot PC (Python / ROS2 optional) -> USB serial -> Arduino -> servo presses spray nozzle

Right now the system is designed around one servo connected to Arduino pin 9, controlled via serial commands:
- `S` = start (press nozzle)
- `R` = stop (release nozzle)
- `P<ms>` = pulse for `<ms>` milliseconds (press, hold, release)


1) Hardware Setup

Servo wiring (typical)
- Servo signal → Arduino D10
- Servo V+ → stable 5V supply (recommended external if servo is strong)
- Servo GND → Arduino GND
- If servo uses external 5V: share ground between Arduino and external supply.

> If you power the servo from a weak source (like a rectangular 9V battery through Arduino), you can get resets or no motion.

---

2) Serial Protocol

All commands are single-letter with an optional number, newline-terminated from Python.

Commands
- `S\n`  
  Start spray: sweep servo from OFF → ON.

- `R\n`  
  Stop spray: sweep servo from ON → OFF.

- `P3000\n`  
  Pulse: sweep OFF→ON, hold for 3000 ms, sweep ON→OFF.  
  If you send `P` (no number), Arduino uses a default duration (3000 ms).


Replies
Arduino replies with:
- `OK\n` for valid commands
- `ERR\n` for invalid commands




1) **Build + source**
```bash
cd ~/spray_ws
source /opt/ros/*/setup.bash
colcon build --packages-select spray_control
source install/setup.bash


// 2. Run the Node
 
Command: ros2 run spray_control spray_node --ros-args -p port:=/dev/ttyACM0 -p baud:=115200


3. 
Trigger Spray
ros2 service call /spray/start std_srvs/srv/Trigger "{}"
ros2 service call /spray/stop  std_srvs/srv/Trigger "{}"
ros2 service call /spray/pulse std_srvs/srv/Trigger "{}"



Hardware note (important)

The servo must physically press/release the nozzle across its full usable range. In practice, tune your code so the servo uses the full 0°→180° sweep (or as close as your servo supports) to reliably actuate the spray.