This project comprises software which was used for the Machine Vision and logic contorl for an autonomous mobile robot within a hamster wheel. MQTT Explorer and Raspberry Pi were utilised for a wireless communication network between the laptop and Obot. The software conducts the following:

- ArUco code detection
- Camera calibration and initialisation of coordinate system
- Robotic control logic, inlcuding:
- Autonomous detection of orientation of robot
- Algorithm to calculate quickest route to visit all stations (ArUco markers) whilst avoiding obstacles
- Live movement logic depending on live feedback of machine vision
- Logic for interacting with each station
- Live 'fuel level' of robot depending on speed and distance travelled, detected via machine vision
- Communincation protocol between laptop and Raspberry-Pi 3A+ and Raspberry-Pi Zero via MQTT Wi-Fi data transfer
- Obstacle detection via machine vision
- GUI overlay to display live tracking informtation
- Glare reduction (glare due to spherical perspex hamster wheel) utilising gaussian blur and adaptive threshold
