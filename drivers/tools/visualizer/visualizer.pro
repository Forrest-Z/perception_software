#-------------------------------------------------
#
# Project created by QtCreator 2020-09-10T16:03:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = visualizer
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    res.qrc

FORMS += \
    main_window.ui \
    msg_dialog.ui \
    scene_camera_dialog.ui \
    video_images_dialog.ui

HEADERS += \
    abstract_camera.h \
    channel_reader.h \
    fixedaspectratiowidget.h \
    free_camera.h \
    grid.h \
    main_window.h \
    msg_dialog.h \
    plane.h \
    pointcloud.h \
    radarpoints.h \
    renderable_object.h \
    scene_camera_dialog.h \
    scene_viewer.h \
    target_camera.h \
    texture.h \
    treewidget.h \
    video_images_dialog.h \
    video_image_viewer.h

SOURCES += \
    abstract_camera.cc \
    fixedaspectratiowidget.cc \
    free_camera.cc \
    grid.cc \
    main.cc \
    main_window.cc \
    msg_dialog.cc \
    plane.cc \
    pointcloud.cc \
    radarpoints.cc \
    renderable_object.cc \
    scene_camera_dialog.cc \
    scene_viewer.cc \
    target_camera.cc \
    texture.cc \
    treewidget.cc \
    video_images_dialog.cc \
    video_image_viewer.cc

DISTFILES += \
    CMakeLists.txt
