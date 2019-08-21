
DEFINES *= use_depthcamera


CONFIG(release, debug|release) {

    # на некоторых платформах в CONFIG по умолчанию включён и release и debug
    # поэтому
    CONFIG -= debug

    DEFINES += QT_NO_DEBUG_OUTPUT

    LIBS_DIR = ../$$TARGET/3rdparty/sortnumbers/lib/
    INCLUDEPATH *= ../$$TARGET/3rdparty/sortnumbers/include/
}

CONFIG(debug, debug|release) {

    # на некоторых платформах в CONFIG по умолчанию включён и release и debug
    # поэтому
    CONFIG -= release

    # при отладке ищём библиотеки в каталоге libs домашнего каталога текущего пользователя
    LIBS_DIR = $$system(echo $HOME)/libs

    unix: !android: LIBS_DIR = $$LIBS_DIR/linux/

    INCLUDEPATH *= ../sort_numbers/
}


LIBS += -L$$LIBS_DIR
LIBS += -lsortNumbers

INCLUDEPATH *= 3rdparty/realsense2/include

LIBS *= -L"../tmpQtWgtsCamera/3rdparty/realsense2/lib/" -lrealsense2
