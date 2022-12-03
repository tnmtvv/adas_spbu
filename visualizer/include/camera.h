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
        [[maybe_unused]] double speedAtMomentInTime{};
        cv::Mat perturbation;
    public:

        [[maybe_unused]] explicit camera(cv::Mat internalCameraParameters);
        [[maybe_unused]] void displayModelPoints(const std::shared_ptr<models>& models);
        [[maybe_unused]] cv::Mat getCameraPlane();
        [[maybe_unused]] void moveAroundTheCircle(float angle);
        [[maybe_unused]] void clear();

        [[maybe_unused]] virtual void moveAlongTheRoad() = 0;
        [[maybe_unused]] std::vector<cv::Point2d> reverseProject(const std::shared_ptr<plane> &plane, const std::vector<cv::Point2i>& vector);
        [[maybe_unused]] void displayPoints(const std::vector<cv::Point3d> &points);
    };
}

#endif
