cmake_minimum_required(VERSION 2.8)

project(adas_spbu)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


find_package( OpenCV REQUIRED )
find_package( OpenPose REQUIRED )
find_package( Threads REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} ${OpenPose_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp src/falsePositiveFilter.hpp src/falsePositiveFilter.cpp )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${OpenPose_LIBS})