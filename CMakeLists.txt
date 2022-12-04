cmake_minimum_required(VERSION 3.5)

project(Tracking LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

if(ANDROID)
  add_library(Tracking SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
else()
  add_executable(Tracking
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    MyTracker.cpp
    MyTracker.h
    trackers/Tracker.cpp
    trackers/Tracker.h
    trackers/OpticalFlowTracker.cpp
    trackers/OpticalFlowTracker.h
    trackers/KCFTracker.cpp
    trackers/KCFTracker.h
    trackers/CSRTTracker.cpp
    trackers/CSRTTracker.h
    state.cpp
    state.h
          WindowState.h)
endif()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(Tracking PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${OpenCV_LIBS})
