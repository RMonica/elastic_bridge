cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(elastic_bridge)

###################################
###################################
find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  std_msgs
  genmsg
  tf_conversions
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  eigen_conversions
  pcl_conversions
  init_fake_opengl_context
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
  DIRECTORY msg
  FILES FrameState.msg
)

add_action_files(
  DIRECTORY action
  FILES downloadPointcloud2.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs sensor_msgs std_msgs geometry_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs sensor_msgs std_msgs geometry_msgs
)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if(NOT EXISTS "/opt/ElasticFusion/Core")
  message(WARNING "elastic_bridge: ElasticFusion not found, package won't be built.")
  return()
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(EFUSION_INCLUDE_DIR "/opt/ElasticFusion/Core/src")
set(EFUSION_LIBRARY "/opt/ElasticFusion/Core/build/libefusion.so")

find_package(efusion REQUIRED)

find_package(Pangolin REQUIRED CONFIG PATHS "/opt/ElasticFusion/deps/Pangolin/build/src/")

find_package(ZLIB REQUIRED)
find_package(CUDA REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elastic_bridge
  CATKIN_DEPENDS message_filters roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EFUSION_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${GUI_INCLUDE_DIR}
)
#######

link_directories(${Pangolin_LIBRARY_DIRS} ${EFUSION_LIBRARY_DIRS} ${OPENNI2_INCLUDE_DIRS})


############################################ elastic node

add_executable(elastic_node
  src/elastic_node.cpp
)

target_link_libraries(elastic_node
  ${catkin_LIBRARIES} 
  ${EFUSION_LIBRARIES} 
  ${Pangolin_LIBRARIES}
  ${ZLIB_LIBRARY}
  ${CUDA_LIBRARIES}
  ${PCL_COMMON_LIBRARIES} 
  ${PCL_IO_LIBRARIES}
)

add_dependencies(
  elastic_node
  ${elastic_bridge_EXPORTED_TARGETS}
)
############################################ action client


add_executable(save_pcl src/save_pcl.cpp)
target_link_libraries(
  save_pcl 
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES} 
  ${PCL_IO_LIBRARIES}
)
add_dependencies(
  save_pcl
  ${elastic_bridge_EXPORTED_TARGETS}
)

