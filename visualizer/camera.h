#ifndef VISUALIZER_CAMERA_H
#define VISUALIZER_CAMERA_H
#include "models.h"

namespace models {
    class [[maybe_unused]] camera: public models {
    private:

        // Плоскость камеры
        cv::Mat cameraPlane;

        // Внутренние параметры камеры
        cv::Mat internalCameraParameters;

        // Вектор моделей, которые будут проецироваться на плоскость камеры
        std::vector<models*> model;
    public:

        // Конструктор
        [[maybe_unused]] explicit camera(cv::Mat internalCameraParameters);

        // Функция для проеции точек моделей на плоскость камеры
        [[maybe_unused]] cv::Mat displayModelPoints(std::vector<models*> models);

        [[maybe_unused]] cv::Mat getCameraPlane();

        // Функция для движения камеры по окружности
        [[maybe_unused]] void moveAroundTheCircle(double angle);
    };
}

#endif //VISUALIZER_CAMERA_H
