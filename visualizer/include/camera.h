#ifndef VISUALIZER_CAMERA_H
#define VISUALIZER_CAMERA_H

#include "models.h"
#include "plane.h"
namespace models {
    class [[maybe_unused]] camera: public models {
    private:
        cv::Mat cameraPlane;
        cv::Mat internalCameraParameters;
        cv::Mat inverseInternalCameraParameters;
        std::vector<std::shared_ptr<models>> model;
    public:

        [[maybe_unused]] explicit camera(cv::Mat internalCameraParameters);

        // Функция для проеции точек моделей на плоскость камеры
        [[maybe_unused]] void displayModelPoints(const std::shared_ptr<models>& models);

        [[maybe_unused]] cv::Mat getCameraPlane();

        [[maybe_unused]] void moveAroundTheCircle(float angle);
        [[maybe_unused]] void clear();

        [[maybe_unused]] std::vector<cv::Point2d> reverseProject(const std::shared_ptr<plane> &plane, const std::vector<cv::Point2i>& vector);

        [[maybe_unused]] void displayPoints(const std::vector<cv::Point3d> &points);
    };
}

#endif
