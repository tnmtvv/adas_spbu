cmake_minimum_required(VERSION 3.23)
project(VisibilityEstimator)

set(CMAKE_CXX_STANDARD 23)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} src/contourDetector.cpp src/contoursMerger.cpp src/distanceEstimator.cpp src/imageFilter.cpp src/main.cpp src/contourDrawer.cpp src/contoursOrdering.cpp src/edgeDetector.cpp src/inversePerspectingMapping.cpp src/contourDetector.h src/contoursMerger.h src/distanceEstimator.h src/imageFilter.h src/contourDrawer.h src/contoursOrdering.h src/edgeDetector.h src/inversePerspectiveMapping.h)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})