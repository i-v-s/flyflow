TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH +=  C:/src/googletest/include \
                C:/src/eigen

SOURCES += test.cpp \
    ../kalman.cpp \
    ../sequenceloop.cpp \
    ../timefusion.cpp \
    ../sequencestd.cpp \
    ../copterimu.cpp \
    ../ellipsoidcalibrator.cpp

LIBS += -LC:/src/build/gtest-mingw32 -lgtest

HEADERS += \
    ../kalman.h \
    ../sequenceloop.h \
    ../timefusion.h \
    ../sequencestd.h \
    ../copterimu.h \
    ../ellipsoidcalibrator.h
