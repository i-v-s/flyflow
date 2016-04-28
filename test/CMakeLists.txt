project(flyflow-test)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ../cmake/)

find_package(GTEST REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
	${GTEST_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

ADD_DEFINITIONS(-std=c++11)

add_executable(flyflow_test test.cpp 
	../conveyor.cpp 
	../frame.cpp 
	../visualizer.cpp 
	../jacobi.cpp
	../kalman.cpp ../kalman.h
)

target_link_libraries(flyflow_test ${GTEST_LIBRARIES} ${OpenCV_LIBS})
#set_property(TARGET flyflow_test PROPERTY CXX_STANDARD 14)
