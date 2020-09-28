

## Installation instructions
Install Ubuntu 18.04 and ROS Melodic. Install these dependencies:
```
$ sudo apt install python-catkin-tools python-wstool libgoogle-glog-dev ros-melodic-mav-msgs
$ sudo apt install libvulkan-dev libglm-dev
``` 

Create a `catkin_ws` folder:
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
```
Set-up the workspace:
```
$ catkin init
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin config --merge-devel
```

Clone the dependencies:
```
$ cd ~/catkin_ws/src
$ wstool init
$ wstool merge vulkan_glasses_for_robots/dependencies.rosinstall
$ wstool up -j8
```  

Then build the workspace:
```
$ cd ~/catkin_ws
$ catkin build
```  



