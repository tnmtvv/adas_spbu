#ifndef VISUALIZER_CAMERA_H
#define VISUALIZER_CAMERA_H

#include "models.h"

namespace models {
    class camera: public models {
    private:
        cv::Mat cameraPlane;
        cv::Mat internalCameraParameters;
        std::vector<std::shared_ptr<models>> model;
    public:

        explicit camera(cv::Mat internalCameraParameters);

        // Функция для проеции точек моделей на плоскость камеры
        void displayModelPoints(const std::shared_ptr<models>& models);

        cv::Mat getCameraPlane();
        void moveAroundTheCircle(float angle);
        void clear();
    };
}

#endif
