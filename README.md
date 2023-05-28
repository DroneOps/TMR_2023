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

### Install catkin tools
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
### Create ROS workspace
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
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
### Clone
Inside your workspace:
```
git clone https://github.com/DroneOps/TMR_2023.git
```
### Install custom TelloPy
```
cd ~/tello_workspace/TMR_2023/TelloPy
sudo python setup.py install
```
ROS Dependencies
```
cd ~/tello_workspace/TMR_2023/ROS/tello_catkin_ws/
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
### Compile Orbslam2 and Custom Packages
```
cd ~/tello_workspace/TMR_2023/ROS/tello_catkin_ws/
catkin init
catkin clean
catkin build
```
Might return error when compiling for the first time, simply repeat catkin build command
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
