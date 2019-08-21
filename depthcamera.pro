
QT       -= gui

TEMPLATE = lib

CONFIG += c++17

# выбираем камеры с которых надо получить данные
CONFIG += use_realsense
#CONFIG += use_myntai
#CONFIG += use_structuresensor


SOURCES += \
        depthcamera.cpp

HEADERS += \
        depthcamera.h

# принимаем за правило, что созданные нами библиотеки
# должны оказаться в каталоге libs домашнего каталога текущего пользователя
LIBS_DIR = $$system(echo $HOME)/libs

# результирующий файл библиотеки должен оказаться в
unix: !android: DESTDIR = $$LIBS_DIR/linux/

CONFIG(use_realsense) {
    SOURCES *= realsensecamera.cpp
    HEADERS *= realsensecamera.h
    INCLUDEPATH *= 3rdparty/realsense2/include
    LIBS *= -L$$system(echo $HOME)/projects/depthcamera/3rdparty/realsense2/lib -lrealsense2
}

CONFIG(use_myntai) {
# однажды :)
}
CONFIG(use_structuresensor) {

}

DISTFILES += \
    depthcamera.pri
