# Obot
This projectMachine Vision and logic software for autonomous robot. Utilising MQTT Explorer and Raspberry Pi for communication network.
The software conducts the following:
  - ArUco code detection
  - Camera calibration and initialisation of coordinate system
  - Robotic control logic, inlcuding:
      - Autonomous detection of orientation of robot
      - Quiqest route to visit all stations (ArUco markers)
      - Live movement logic depending on live feedback of machine vision
  - Logic for interacting with each station
  - Live 'fuel level' of robot depending on speed and distance travelled, detected via machine vision
  - Communincation protocol between laptop and Raspberry-Pi 3A+ and Zero via MQTT Wi-Fi data transfer
  - Obstacle detection, including
  - Edge detection and shape filtering
