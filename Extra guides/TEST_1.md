# Test 1: setup and connection

If you have followed the setup guide correctly you should be almost ready to connect to the drone and perform a quick test of takeoff and landing.

### Prerequisites:
Follow the main README.md to completion.

### Get a wifi adapter
Some computers, because of their wifi adapter incompatibility with Ubuntu 18.04 and due to missing drivers, will not be able to get a wifi connection. We must have a wifi connection to communicate with the drone. We recommend buying an adapter like the TP-Link Archer T2U Nano (https://a.co/d/fSW9fIf), because we have tested that it works.


Note: This recommended adapter's drivers can be acquired following: https://askubuntu.com/questions/1149117/tp-link-ac600-archer-t2u-nano-driver-for-ubuntu-18-04

### Modify routes to .csv path file
```
cd /home/<user>/tello_workspace/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/src
```
Inside wall_land.py, replace '/home/droneops/Documents/TMR_2023/ROS/tello_catkin_ws/src/flock/flock_driver/src/test.csv' with the appropriate path in your computer.

Inside data_logger.py, replace '/home/droneops/Documents/Datasets/OrbSLAM_dataset/' with the appropriate path in your computer.

### Connect and roslaunch
Turn on the drone and connect to it via wifi.
```
source devel/setup.bash
roslaunch flock_driver wall_node.launch
```
Observe that windows open up, focus your attention in the main terminal, wait for processes and then click enter to initialize flight. Switch you attention to the override terminal, be ready to force landing. Wait 15 seconds, click enter in the override terminal, wait for landing, and interrupt the running process in the main terminal.
You have successfully connected and flown the drone.
