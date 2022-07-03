# Brick_Feeder

## Requirements
- EPS32.
- Visual Studio Code with PlatformIO extension installed.
## PlatformIO requirements
- Arduino framework installed
- Micro-ros-arduino installed

## How to install and setup
- Clone the repository
- Change the agent_ip to the ip address of the pc that will run the docker agent.
  - https://github.com/kajMork/Brick_Feeder/blob/0dc68f77e6fc0c8e5619c6593ff8418edf9964ec/src/main.cpp#L62
- Change the SSID_name to the SSID name of the router that the agent pc is connected to.
  - https://github.com/kajMork/Brick_Feeder/blob/0dc68f77e6fc0c8e5619c6593ff8418edf9964ec/src/main.cpp#L63
- Change the SSID_psw to the password of the router.
  - https://github.com/kajMork/Brick_Feeder/blob/0dc68f77e6fc0c8e5619c6593ff8418edf9964ec/src/main.cpp#L64
- Build and upload the code to the ESP32.
- Now follow this [Wiki tutorial](https://github.com/kajMork/Brick_Feeder/wiki/How-to-make-ROS2-and-micro-ros-communicate-together.) on how to start the agent (tutorial is only tested for Ubuntu 20.04).

For namespacing, change the third parameter of rclc_node_init_default from "" to your namespace.
https://github.com/kajMork/Brick_Feeder/blob/0dc68f77e6fc0c8e5619c6593ff8418edf9964ec/src/main.cpp#L260
![alt text](https://github.com/kajMork/Brick_Feeder/blob/main/docs/Brick_feeder.jpg?raw=true)
