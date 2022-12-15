# CMakeList.txt: проект CMake для traffic_signs_detection; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

project(traffic_signs_detection)

# Добавьте источник в исполняемый файл этого проекта.
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable (traffic_signs_detection "main.cpp" "main.h" "imageSegmentation.h" "imageSegmentation.cpp" "detectorPolygons.cpp" "detectorPolygons.h")

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# TODO: Добавьте тесты и целевые объекты, если это необходимо.
