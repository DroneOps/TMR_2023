# TMR_2023



## Setup

Note: in the following instructions, we named our workspace tello_workspace, you can copy and paste all commands if you name your worskpace the same, othwerwise, replace tello_workspace with yours.

### Prerequisites:
Fresh install of ubuntu 18.04


### Install ROS Melodic 
Follow this page
http://wiki.ros.org/melodic/Installation/Ubuntu



### Install the following version of ffmpeg:

```
wget "http://archive.ubuntu.com/ubuntu/pool/universe/f/ffmpeg/ffmpeg_3.4.11-0ubuntu0.1_amd64.deb"
dpkg -i ffmpeg_3.4.11-0ubuntu0.1_amd64.deb
```

### Install the following version of av library:
```
pip install av==0.3.3
```

### Install catking tools
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```
### Install Eigen3
```
sudo apt install libeigen3-dev
```
### Install Python PIL
```
sudo apt-get install python-imaging-tk
```
### Install Pangolin v0.6
Inside your workspace:
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout tags/v0.6
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt-get install libxkbcommon-dev
mkdir build
cd build
cmake ..
make
```
### Install h264decoder v1
Inside your workspace:
```
git clone https://github.com/DaWelter/h264decoder.git
cd h264decoder
git checkout tags/v1
```
Inside h264decoder.cpp replace PIX_FMT_RGB24 with AV_PIX_FMT_RGB24
Then:
```
mkdir build
cd build
cmake ..
make
```
copy to python path
```
sudo cp ~/tello_workspace/h264decoder/libh264decoder.so /usr/local/lib/python2.7/dist-packages
```
### Install Tello_ROS_ORBSLAM
Inside your workspace
```
git clone https://github.com/tau-adl/Tello_ROS_ORBSLAM.git
```
Install their version of TelloPY
```
cd ~/tello_workspace/Tello_ROS_ORBSLAM/TelloPy
sudo python setup.py install
```
ROS Dependencies
```
cd ~/tello_workspace/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### Install Orbslam2
Change the CMakeLists.txt on ~/ros_workspace/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/src/orb_slam_2_ros/CMakeLists.txt  to the one included in this folder.
```
cd ~/tello_workspace/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
catkin init
catkin clean
catkin build
```
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Clone
Inside your workspace:
```
git clone https://github.com/DroneOps/TMR_2023.git
```

