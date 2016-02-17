project(flyflow)
cmake_minimum_required(VERSION 2.8)

SET(USE_ROS TRUE)

find_package(OpenCV 3.0.0 REQUIRED)

if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()

if(USE_ROS)
  add_definitions(-DUSE_ROS)

  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge)
    catkin_package(
      DEPENDS OpenCV 3.0.0
      CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
      #INCLUDE_DIRS include
      #LIBRARIES 
  )

  include_directories(
      ${catkin_INCLUDE_DIRS}
  )
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})
