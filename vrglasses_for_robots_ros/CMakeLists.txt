cmake_minimum_required(VERSION 2.8.0)
project(vrglasses_for_robots_ros)

#set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set (CMAKE_CXX_STANDARD 17)
find_package( Vulkan REQUIRED )

find_package( Boost COMPONENTS system filesystem thread REQUIRED )
find_package( OpenCV REQUIRED core highgui )

include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${TINYOBJLOADER_INCLUDE_DIRS} )

include_directories( include )

#hack to add headers in the qtcreator
FILE(GLOB_RECURSE hearderFiles "include/vrglasses_for_robots_ros/*.h")
add_custom_target(headers SOURCES ${hearderFiles})

############
# BINARIES #
############

cs_add_executable(${PROJECT_NAME}
  src/ros_main.cc
  src/vrglasses_node.cc
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${Vulkan_LIBRARY} )

##########
# EXPORT #
##########

cs_install()
cs_install_scripts(scripts/pose_publisher.py)
cs_install_scripts(scripts/ortho_model_top_view.py)
cs_install_scripts(scripts/pose_publisher_viewpoints.py)
cs_export()
