cmake_minimum_required(VERSION 3.23)
project(Visualizer)

set(CMAKE_CXX_STANDARD 23)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} main.cpp coordinateSystem.cpp coordinateSystem.h models.cpp models.h camera.cpp camera.h plane.cpp plane.h cube.cpp cube.h vertex.cpp vertex.h objParser.cpp objParser.h main.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})