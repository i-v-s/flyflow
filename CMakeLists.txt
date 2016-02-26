project(flyflow)
cmake_minimum_required(VERSION 2.8)

#SET(USE_ROS TRUE)

find_package(OpenCV 3.0.0 REQUIRED)
#find_package(gtest)
#find_package(Eigen3 REQUIRED)

if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()

if(USE_ROS)
  add_definitions(-DUSE_ROS)

  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge)
    catkin_package(
      DEPENDS OpenCV 3.0.0 Eigen
      CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
      #INCLUDE_DIRS include
      #LIBRARIES 
  )

  include_directories(
      #${Eigen3_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_executable(flyflow main.cpp conveyor.cpp frame.cpp visualizer.cpp imagestat.cpp)
target_link_libraries(flyflow ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(flyflow_test test.cpp conveyor.cpp frame.cpp visualizer.cpp )#jacobi.cpp)
target_link_libraries(flyflow_test gtest ${OpenCV_LIBS} ${catkin_LIBRARIES})
