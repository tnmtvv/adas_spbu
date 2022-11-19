#include "camera.h"
#include <utility>

namespace models {

    [[maybe_unused]] camera::camera(cv::Mat internalCameraParameters) : models() {
        this->internalCameraParameters = std::move(internalCameraParameters);
        this->cameraPlane = cv::Mat (1080, 1920, CV_8UC1, cv::Scalar(0));
    }

    [[maybe_unused]] cv::Mat camera::displayModelPoints(std::vector<models*> models) {
        for (auto & object : models) {
            auto globalPoints = object->getGlobalPoints();
            auto localPoints = object->getLocalPoints();
            std::map<int, cv::Point2l> projection;
            for (int i = 0; i < globalPoints.size(); i++) {
                cv::Mat localCameraPoints = internalCameraParameters * this->coordinateSystem->moveToLocalCoordinates(globalPoints[i]);
                /*
                * тут отсчение невидимых точек камеры, но функция работает не так как надо
                */
                int x = (int) (localCameraPoints.at<double>(0, 0) / localCameraPoints.at<double>(2, 0));
                int y = (int) (localCameraPoints.at<double>(1, 0) / localCameraPoints.at<double>(2, 0));
                projection[i] = cv::Point2d(x, y);
            }

            auto polygons = object->getIndexes();
            for (auto & polygon : polygons) {
                for (int k = 0; k < polygon.size(); k++) {
                    int h = k;
                    k = k + 1;
                    while (k != polygon.size()) {
                        cv::line(cameraPlane, projection[polygon[h]], projection[polygon[k]], 255, 1,
                                 cv::LINE_8);
                        k++;
                    }
                    k = h;
                }
            }
        }
        cv::Mat mat;
        cameraPlane.copyTo(mat);
        cameraPlane = cv::Scalar(0);
        return mat;

        /*
        auto newGraph = model->getGraph();
        for(auto iter = newGraph.begin(); iter != newGraph.end(); iter++)
        {
            auto vector = iter->second;
            for (int i = 0; i < vector.size(); i++) {
                cv::line(cameraPlane, projection[iter->first], projection[{vector[i].x, vector[i].y, vector[i].z}], 255, 2, cv::LINE_8);
            }
        }*/
    }

    [[maybe_unused]] [[maybe_unused]] cv::Mat camera::getCameraPlane() {
        return cameraPlane;
    }

    [[maybe_unused]] void camera::moveAroundTheCircle(double angle) {
        auto coordinates = this->getCoordinatesOfCenter();
        auto rad = coordinates[0] * coordinates[0] + coordinates[1]*coordinates[1] + coordinates[2]*coordinates[2];
        auto tsquare = sqrt(2*rad*(1 - cos(angle)));
        this->moveInLocalCoordinates(cv::Vec3d(tsquare*cos(angle / 2), 0, tsquare*sin(angle / 2 )));
        this->rotate(angle, yAxis);
    }
} // Models