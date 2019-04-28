TARGET = depthcamera
TEMPLATE = lib

CONFIG += c++1z

DEFINES += DEPTHCAMERA_LIBRARY
DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    myntcamera.cpp \
    realsensecamera.cpp

HEADERS += \
        depthcamera_global.h \ 
    myntcamera.h \
    realsensecamera.h \
    idepthcamera.h

INCLUDEPATH +=  /usr/local/include/pcl-1.9

LIBS += -lmynteye -lrealsense2 -lpcl_common

unix {
    target.path = /usr/lib
    INSTALLS += target
}
