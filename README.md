

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

## Running
To be able to visualize some rendering is necessary to have a node publishing camera poses on the topic subscribed by the vrglasses_for_robots_ros node. The script vrglasses_for_robots_ros/scripts/pose_publisher.py is an example for a simple camera pose publisher.

### Single model
roslaunch vrglasses_for_robots_ros simple_example.launch file_obj:=/home/lucas/Downloads/textures/oakland_library_semantics/oakland-nj-public-library_v1.r2.obj file_texture:=/home/lucas/Downloads/textures/oakland_library_semantics/oakland_library_texture8k_roof_rgbs.tga

### Multiple Models
roslaunch vrglasses_for_robots_ros multiple_model_example.launch model_folder:=/home/lucas/Downloads/textures model_list_file:=/media/secssd/catkin_ws/src/vulkan_glasses_for_robots/vrglasses_for_robots_ros/models/example_model_def_list.txt  model_pose_file:=/media/secssd/catkin_ws/src/vulkan_glasses_for_robots/vrglasses_for_robots_ros/models/example_model_poses_list.txt

### Dynamic models
To have moving models, run the following file:
```
$ roslaunch vrglasses_for_robots_ros dynamic_model_example.launch \
model_folder:=/path/to/folder/with/meshes \
model_list_file:=/path/to/file/with/models.txt \
dynamic_model_pose_file:=/path/to/trajectories/with/models.txt
```
where the file are as follows (**per row**, respect white spaces!):
* `model_list_file`: model_name;relative/path/to/model.obj;/relative/path/to/model/texture.png
* `dynamic_model_pose_file`: 
	* Static models: `model_name;0;yaw x y z`
	* Dynamic models: `model_name;velocity;num_repetitions;yaw0 x0 y0 z0;yaw1 x1 y1 z1;...`
	
For the dynamic models, the single model is repeated `num_repetitions` times along the trajectory (models are equally spaced along path).

