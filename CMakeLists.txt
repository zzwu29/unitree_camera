cmake_minimum_required(VERSION 3.0.2)
project(unitree_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

# 设置UnitreeCameraSDK的路径（需要根据实际安装路径调整）
set(UNITREE_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk/UnitreecameraSDK")

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
# set(LIBRARY_OUTPUT_PATH ${UNITREE_SDK_PATH}/lib/amd64/)
link_directories(${UNITREE_SDK_PATH}/lib/amd64/)
else()
# set(LIBRARY_OUTPUT_PATH ${UNITREE_SDK_PATH}/lib/arm64/)
link_directories(${UNITREE_SDK_PATH}/lib/arm64/)
endif()

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${UNITREE_SDK_PATH}/include
)

add_executable(unitree_camera_node src/unitree_camera_node.cpp)

target_link_libraries(unitree_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  unitree_camera
  tstc_V4L2_xu_camera udev
  systemlog
)
