project(flyflow-test)
cmake_minimum_required(VERSION 2.8)

ADD_DEFINITIONS(-std=c++14)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ../cmake/)

if(NOT ${GTEST_FOUND})
    find_package(GTEST REQUIRED)
endif()

if(NOT EIGEN3_INCLUDE_DIR)
    find_package(Eigen3 REQUIRED)
endif()

#message( STATUS "EIGEN3_INCLUDE_DIR: " ${EIGEN3_FOUND})

#find_package(OpenCV)

include_directories(
	${GTEST_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

set(SOURCES test.cpp
            ../normdist.cpp     ../normdist.h
            ../kalman.cpp       ../kalman.h
            ../sequenceloop.cpp ../sequenceloop.h
            ../timefusion.cpp
            ../sequencestd.cpp
            ../ellipsoidcalibrator.cpp
            ../imu.cpp
            ../imukalman.cpp
            ../vectpl.cpp
)

if(OpenCV_FOUND)
    set(SOURCES ${SOURCES}
        ../conveyor.cpp
        ../frame.cpp
        ../visualizer.cpp
        ../jacobi.cpp


    )
endif()

add_executable(flyflow_test ${SOURCES})


target_link_libraries(flyflow_test ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread)
#set_property(TARGET flyflow_test PROPERTY CXX_STANDARD 14)
