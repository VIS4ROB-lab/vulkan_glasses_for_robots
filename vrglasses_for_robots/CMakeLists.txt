cmake_minimum_required(VERSION 2.8.0)
project(vrglasses_for_robots)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set (CMAKE_CXX_STANDARD 17)
find_package( Vulkan REQUIRED )

find_package( Boost COMPONENTS system filesystem thread REQUIRED )

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${TINYOBJLOADER_INCLUDE_DIRS} )

include_directories( include )

#hack to add headers in the qtcreator
FILE(GLOB_RECURSE hearderFiles "include/vrglasses_for_robots/*.h")
add_custom_target(headers SOURCES ${hearderFiles})

add_custom_target(
    vrglasses4robots_shader.frag.spv
    #ALL
    COMMAND glslangValidator -V -o vrglasses4robots_shader.frag.spv shaders/vrglasses4robots_shader.frag
    SOURCES shaders/vrglasses4robots_shader.frag
    )


#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}_lib
  src/VulkanTools.cc
  src/vulkan_renderer.cc
)
target_link_libraries(${PROJECT_NAME}_lib   ${Vulkan_LIBRARY} )


############
# BINARIES #
############

cs_add_executable(${PROJECT_NAME}_fs
  src/fs_main.cc
  src/visim_processor.cc
  src/data_source.cc
)

target_link_libraries(${PROJECT_NAME}_fs glog gflags  ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Vulkan_LIBRARY} ${PROJECT_NAME}_lib)

##########
# EXPORT #
##########

cs_install()
cs_export()
