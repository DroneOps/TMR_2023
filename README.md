# TMR_2023



## Setup

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
```
cd ~/ROS/
git clone https://github.com/stevenlovegrove/Pangolin.git
git checkout tags/v0.6
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt-get install libxkbcommon-dev
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build
```
### Install h264decoder v1
```
cd ~/ROS/
git clone https://github.com/DaWelter/h264decoder.git
git checkout tags/v1
Inside h264decoder.cpp replace PIX_FMT_RGB24 with AV_PIX_FMT_RGB24
mkdir build
cd build
cmake ..
make
```
copy to python path
```
sudo cp ~/ROS/h264decoder/libh264decoder.so /usr/local/lib/python2.7/dist-packages
```
### Install Tello_ROS_ORBSLAM
```
cd ~
mkdir ROS
cd ROS
git clone https://github.com/tau-adl/Tello_ROS_ORBSLAM.git
Install their version of TelloPY
cd ~/ROS/Tello_ROS_ORBSLAM/TelloPy
sudo python setup.py install
ROS Dependencies
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### Install Orbslam2
Change the CMakeLists.txt on ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/src/orb_slam_2_ros/CMakeLists.txt  to the one included in this folder.
```
cd ~/ROS/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
catkin init
catkin clean
catkin build
```
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Clone
```
cd ~/ROS
git clone https://github.com/DroneOps/TMR_2023.git
```

