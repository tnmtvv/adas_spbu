cmake_minimum_required(VERSION 3.23)
project(Visualizer)

set(CMAKE_CXX_STANDARD 23)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} source/coordinateSystem.cpp source/models.cpp source/camera.cpp source/plane.cpp
        source/cube.cpp source/objParser.cpp source/main.cpp source/parabola.cpp source/segment.cpp source/StrategyOfAddBorders.cpp source/ParabolaStrategy.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/coordinateSystem.h include/models.h
        include/camera.h include/plane.h include/cube.h include/parabola.h include/StrategyOfAddBorders.h
        include/ParabolaStrategy.h include/segment.h include/objParser.h lol)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})