#include "../include/camera.h"
#include <utility>

namespace models {

    camera::camera(cv::Mat internalCameraParameters) : models() {
        this->internalCameraParameters = std::move(internalCameraParameters);
        this->cameraPlane = cv::Mat (1080, 1920, CV_8UC1, cv::Scalar(0));
    }

    void camera::displayModelPoints(const std::shared_ptr<models>& models) {
        std::vector<cv::Point2d> imagePoints;
        cv::Mat distCoeffs(5, 1, CV_64FC1, cv::Scalar(0));
        std::vector<int> globalPoints;
        auto modelPoints = models->getGlobalPoints();
        for (int i = 0; i < modelPoints.size(); i++) {
            if (this->coordinateSystem->moveToLocalCoordinates(modelPoints[i]).at<double>(2, 0) < 0) {
                globalPoints.emplace_back(i);
            }
        }

        cv::projectPoints(models->getGlobalPoints(), this->getCoordinateSystem()->getRotationMatrix(),
                          this->getCoordinateSystem()->getCoordinatesOfCenter(), internalCameraParameters, distCoeffs,
                          imagePoints);

        auto polygons = models->getIndexes();
        for (auto index : globalPoints) {
            for (auto &polygon: polygons) {
                for (int k = 0; k < polygon.size(); k++) {
                    if (polygon[k] == index){
                        polygon.erase(polygon.begin() + k);
                        k--;
                    }
                }
            }
        }

        for (auto &polygon: polygons) {
            for (int k = 0; k < polygon.size(); k++) {
                int h = k;
                k = k + 1;
                while (k < polygon.size()) {
                    auto firstPoint = cv::Point2i((int) imagePoints[polygon[h]].x, (int) imagePoints[polygon[h]].y);
                    auto secondPoint = cv::Point2i((int) imagePoints[polygon[k]].x, (int) imagePoints[polygon[k]].y);
                    cv::line(cameraPlane, firstPoint, secondPoint, 255, 1,
                             cv::LINE_8);
                    k++;
                }
                k = h;
            }
        }
    }

    void camera::clear()
    {
        cameraPlane = cv::Scalar(0);
    }

    cv::Mat camera::getCameraPlane() {
        return cameraPlane;
    }

    void camera::moveAroundTheCircle(float angle) {
        auto coordinates = this->getCoordinatesOfCenter();
        auto rad = coordinates[0] * coordinates[0] + coordinates[1]*coordinates[1] + coordinates[2]*coordinates[2];
        auto tsquare = sqrt(2*rad*(1 - cos(angle)));
        this->moveInLocalCoordinates(cv::Vec3d(tsquare*cos(angle / 2), 0, tsquare*sin(angle / 2 )));
        this->rotate(angle, yAxis);
    }
}