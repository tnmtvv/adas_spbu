#include "../include/coordinateSystem.h"

namespace coordinateSystem {

    void coordinateSystem::move(const cv::Vec3d& offset) {
        coordinatesOfCenterOfCoordinateSystem += offset;
    }

    void coordinateSystem::rotate(double angle, Axis axis) {
        auto copyRotationMatrix = rotationMatrix;
        rotationMatrix = generateRotationMatrix(angle, axis) * copyRotationMatrix;
    }

    cv::Mat coordinateSystem::generateRotationMatrix(double angle, Axis axis) {
        switch(axis){
            case xAxis:{
                xAngle += angle;
                cv:: Mat newRotation = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(angle),
                        -sin(angle), 0, sin(angle), cos(angle));
                return newRotation;
            }
            case yAxis:{
                yAngle += angle;
                cv::Mat newRotation = (cv::Mat_<double>(3, 3) << cos(angle), 0, sin(angle), 0, 1, 0,
                        -sin(angle), 0, cos(angle));
                return newRotation;
            }
            case zAxis:{
                zAngle += angle;
                cv::Mat newRotation = (cv::Mat_<double>(3, 3) << cos(angle), -sin(angle), 0,
                        sin(angle), cos(angle), 0, 0, 0, 1);
                return newRotation;
            }
        }

        return {3, 3, CV_64FC1, cv::Scalar(0)};
    }

    cv::Mat coordinateSystem::moveToGlobalCoordinates(const cv::Vec3d& coordinates) {
        return coordinatesOfCenterOfCoordinateSystem + rotationMatrix.t() * coordinates;
    }

    cv::Mat coordinateSystem::moveToLocalCoordinates(const cv::Vec3d& coordinates) {
        return rotationMatrix * (coordinates - coordinatesOfCenterOfCoordinateSystem);
    }

    coordinateSystem::coordinateSystem() {
        coordinatesOfCenterOfCoordinateSystem = cv::Mat(3, 1, CV_64FC1, cv::Scalar(0));
        xAngle = 0;
        yAngle = 0;
        zAngle = 0;
        rotationMatrix = generateRotationMatrix(0, xAxis);
    }

    double coordinateSystem::getAngle(Axis axis) const {
        switch(axis){
            case xAxis:{
                return xAngle;
            }
            case yAxis:{
                return yAngle;
            }
            case zAxis:{
                return zAngle;
            }
        }

        return 0;
    }

    cv::Mat coordinateSystem::getCoordinatesOfCenter() {
        return coordinatesOfCenterOfCoordinateSystem;
    }

    void coordinateSystem::setCoordinatesOfCenter(const cv::Vec3d& vector) {
        coordinatesOfCenterOfCoordinateSystem = cv::Mat(vector);
    }

    [[maybe_unused]] cv::Mat coordinateSystem::getRotationMatrix() {
        return rotationMatrix;
    }
}