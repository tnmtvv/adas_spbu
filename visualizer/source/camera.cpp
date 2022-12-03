#include "../include/camera.h"
#include <utility>

namespace models {

    [[maybe_unused]] camera::camera(cv::Mat internalCameraParameters) : models() {
        this->internalCameraParameters = std::move(internalCameraParameters);
        this->inverseInternalCameraParameters = this->internalCameraParameters.inv();
        std::cout<<this->internalCameraParameters*inverseInternalCameraParameters;
        this->cameraPlane = cv::Mat (1080, 1920, CV_8UC1, cv::Scalar(0));
    }

    [[maybe_unused]] void camera::displayModelPoints(const std::shared_ptr<models>& models) {

        // Знаю, что максимально неэффективно, вместо проверки на то что точка сзади камеры, буду отправлять
        // в эту функцию нужные точки
        std::vector<cv::Point2d> imagePoints;
        cv::Mat distCoeffs(5, 1, CV_64FC1, cv::Scalar(0));
        std::vector<int> globalPoints;
        auto modelPoints = models->getGlobalPoints();
        for (int i = 0; i < modelPoints.size(); i++) {
            if (this->coordinateSystem->moveToLocalCoordinates(modelPoints[i]).at<double>(2, 0) < 0) {
                globalPoints.emplace_back(i);
            }
        }


        for (auto & modelPoint : modelPoints) {
            cv::Mat pip = internalCameraParameters * this->coordinateSystem->moveToLocalCoordinates(cv::Vec3d(modelPoint.x, modelPoint.y, modelPoint.z));
            int x = (int)(pip.at<double>(0 ,0) / pip.at<double>(2 ,0));
            int y = (int)(pip.at<double>(1 ,0) / pip.at<double>(2 ,0));
            imagePoints.emplace_back(x, y);
        }/*
        cv::projectPoints(cv::Mat(models->getGlobalPoints()), this->getCoordinateSystem()->getRotationMatrix(),
                          -this->getCoordinateSystem()->getCoordinatesOfCenter(), internalCameraParameters, distCoeffs,
                          imagePoints);*/

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

    [[maybe_unused]] [[maybe_unused]] void camera::displayPoints(const std::vector<cv::Point3d>& points) {

        std::vector<cv::Point2d> imagePoints;
        std::vector<int> globalPoints;
        if (points.empty())
            return;
        for (const auto & point : points) {
            cv::Mat pip = internalCameraParameters * this->coordinateSystem->moveToLocalCoordinates(cv::Vec3d(point.x, point.y, point.z));
            int x = (int)(pip.at<double>(0 ,0) / pip.at<double>(2 ,0));
            int y = (int)(pip.at<double>(1 ,0) / pip.at<double>(2 ,0));
            if (pip.at<double>(2 ,0) < 0) {
                continue;
            }
            imagePoints.emplace_back(x, y);
        }
        /*
        cv::Mat distCoeffs(5, 1, CV_64FC1, cv::Scalar(0));
        cv::projectPoints(points, this->getCoordinateSystem()->getRotationMatrix(),
                          this->getCoordinateSystem()->getCoordinatesOfCenter(), internalCameraParameters, distCoeffs,
                          imagePoints);*/

        for (int i = 0; i < imagePoints.size() - 1; i++) {
            cv::line(cameraPlane, imagePoints[i], imagePoints[i+1], 255);
        }
    }

    [[maybe_unused]] [[maybe_unused]] std::vector<cv::Point2d> camera::reverseProject(const std::shared_ptr<plane>& plane, const std::vector<cv::Point2i>& vector) {

        // Знаю, что максимально неэффективно, вместо проверки на то что точка сзади камеры, буду отправлять
        // в эту функцию нужные точки
        std::vector<cv::Point2d> answer;
        for (auto& point : vector) {
            auto vec = cv::Vec3d(10*point.x, 10*point.y, 10);
            cv::Mat u = inverseInternalCameraParameters*vec;
            auto pop = this->coordinateSystem->moveToGlobalCoordinates(cv::Vec3d(u));
            auto kek = plane->getCoordinateSystem()->moveToLocalCoordinates(cv::Vec3d(pop));
            answer.emplace_back(kek.at<double>(0, 0),kek.at<double>(1, 0));
        }/*
        std::vector<cv::Point2d> coordinates;
        cv::Mat distCoeffs(5, 1, CV_64FC1, cv::Scalar(0));
        cv::projectPoints(plane->getCoordinatesOfCenter(), this->getCoordinateSystem()->getRotationMatrix(),
                          this->getCoordinateSystem()->getCoordinatesOfCenter(), internalCameraParameters, distCoeffs,
                          coordinates);
        for (auto & j : vector) {
            j.x -= int(coordinates[0].x);
            j.y -= int(coordinates[0].y);
            j.x /= 10;
            j.y /= 10;
            answer.emplace_back(j.x, j.y);
        }*/
        return answer;
    }

    [[maybe_unused]] void camera::clear()
    {
        cameraPlane = cv::Scalar(0);
    }

    [[maybe_unused]] cv::Mat camera::getCameraPlane() {
        return cameraPlane;
    }

    [[maybe_unused]] void camera::moveAroundTheCircle(float angle) {
        auto coordinates = this->getCoordinatesOfCenter();
        auto rad = coordinates[0] * coordinates[0] + coordinates[1]*coordinates[1] + coordinates[2]*coordinates[2];
        auto tsquare = sqrt(2*rad*(1 - cos(angle)));
        this->moveInLocalCoordinates(cv::Vec3d(tsquare*cos(angle / 2), 0, tsquare*sin(angle / 2 )));
        this->rotate(angle, coordinateSystem::yAxis);
    }
}