TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    markerdetector.cpp \
    tcpacceptor.cpp \
    tcpstream.cpp \

LIBS += /usr/local/lib64/librealsense.so\
        -lopencv_core\
        -lopencv_highgui\
        -lopencv_imgproc\
        -lopencv_imgcodecs\
        -lzmq\

HEADERS += \
    markerdetector.h \
    tcpacceptor.h \
    tcpstream.h \

