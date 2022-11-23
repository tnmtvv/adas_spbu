#ifndef VISUALIZER_CAMERA_H
#define VISUALIZER_CAMERA_H

#include "models.h"

namespace models {
    class [[maybe_unused]] camera: public models {
    private:
        cv::Mat cameraPlane;
        cv::Mat internalCameraParameters;
        std::vector<std::shared_ptr<models>> model;
    public:

        [[maybe_unused]] explicit camera(cv::Mat internalCameraParameters);

        // Функция для проеции точек моделей на плоскость камеры
        [[maybe_unused]] void displayModelPoints(const std::shared_ptr<models>& models);

        [[maybe_unused]] cv::Mat getCameraPlane();

        [[maybe_unused]] void moveAroundTheCircle(float angle);

        [[maybe_unused]] void clear();
    };
}

#endif
