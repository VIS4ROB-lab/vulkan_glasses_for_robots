
## Installation instructions no Melodic
### Install Ubuntu 18.04 and ROS Melodic. Install these dependencies:
```
$ sudo apt install python-catkin-tools python-wstool libgoogle-glog-dev ros-melodic-mavros-msgs libgflags-dev
$ sudo apt install libvulkan-dev libglm-dev
``` 

### Create a `catkin_ws` folder:
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
```

### Set-up the workspace:
```
$ catkin init
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin config --merge-devel
```

### Clone the dependencies:
```
$ cd ~/catkin_ws/src
$ wstool init
$ wstool merge vulkan_glasses_for_robots/dependencies.rosinstall
$ wstool up -j8
```  

### Then build the workspace:
```
$ cd ~/catkin_ws
$ catkin build
```  



## Setup Noetic in a docker

### Setting up the Build Environment using Docker

If your operating system doesn't support ROS 1 noetic, docker is a great alternative. 

First, create the image, with the build context at the root of this repo

```Bash
docker build --file docker/Dockerfile --tag vrg-ros1 .
```

You can see the image exists:
```Bash
docker images
>>> REPOSITORY                TAG       IMAGE ID       CREATED        SIZE
>>> vrg-ros1                  latest    5565f845ab4f   2 weeks ago    1.1MB
```

Next, run the image, mounting in the source into a workspace. All the dependencies are now installed.
```Bash
docker run --network=host -it -v $(pwd):/root/catkin_ws/src/vulkan_glasses_for_robots -w /root/catkin_ws vrg-ros1 bash
```
Alternatively there is a run_docker.sh script for display setup.

### Running the Build

Configure the catkin workspace
```Bash
catkin config --extend "/opt/ros/noetic"
catkin config --merge-devel
```

Build the package
```Bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin build -j$(nproc) -l$(nproc) 
```



## Running
To be able to visualize some rendering is necessary to have a node publishing camera poses on the topic subscribed by the vrglasses_for_robots_ros node. The script vrglasses_for_robots_ros/scripts/pose_publisher.py is an example for a simple camera pose publisher.

### Single model
roslaunch vrglasses_for_robots_ros simple_example.launch file_obj:=/home/lucas/Downloads/textures/oakland_library_semantics/oakland-nj-public-library_v1.r2.obj file_texture:=/home/lucas/Downloads/textures/oakland_library_semantics/oakland_library_texture8k_roof_rgbs.tga

### Multiple Models
roslaunch vrglasses_for_robots_ros multiple_model_example.launch model_folder:=/home/lucas/Downloads/textures model_list_file:=/media/secssd/catkin_ws/src/vulkan_glasses_for_robots/vrglasses_for_robots_ros/models/example_model_def_list.txt  model_pose_file:=/media/secssd/catkin_ws/src/vulkan_glasses_for_robots/vrglasses_for_robots_ros/models/example_model_poses_list.txt


