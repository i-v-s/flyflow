TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt
#QMAKE_CXXFLAGS += -finput-charset=UTF-8

INCLUDEPATH +=  C:/src/googletest/include \
                C:/src/eigen

SOURCES += test.cpp \
    ../kalman.cpp \
    ../sequenceloop.cpp \
    ../timefusion.cpp \
    ../sequencestd.cpp \
    ../ellipsoidcalibrator.cpp \
    ../imu.cpp \
    ../imukalman.cpp \
    ../vectpl.cpp

LIBS += -LC:/src/build/gtest-mingw32 -lgtest

HEADERS += \
    ../kalman.h \
    ../sequenceloop.h \
    ../timefusion.h \
    ../sequencestd.h \
    ../ellipsoidcalibrator.h \
    ../imu.h \
    ../imukalman.h \
    ../vectpl.h
