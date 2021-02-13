cmake_minimum_required(VERSION 3.5)

project(track_builder LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(XIMEA REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${XIMEA_INCLUDE_DIRS})

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        ximeacamera.cpp
        ximeacamera.h
        rtthread.cpp
        rtthread.h
        imageviewer.cpp
        imageviewer.h
        imagetools.cpp
        imagetools.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(track_builder
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(track_builder SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(track_builder
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(track_builder PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${OpenCV_LIBS} ${XIMEA_LIBRARY})
