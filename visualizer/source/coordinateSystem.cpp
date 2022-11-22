#include "../include/coordinateSystem.h"

namespace coordinateSystem {

    void coordinateSystem::move(const cv::Vec3d& offset) {
        coordinatesOfCenterOfCoordinateSystem += offset;
    }

    void coordinateSystem::rotate(double angle, Axis axis) {
        rotationMatrix = generateRotationMatrix(angle, axis) * rotationMatrix;
    }

    cv::Mat coordinateSystem::generateRotationMatrix(double angle, Axis axis) {
        cv::Mat matrix(3, 3, CV_64FC1, cv::Scalar(0));
        switch(axis){
            case xAxis:{
                xAngle += angle;
                double data[3][3] = {{1 ,0 , 0},
                                     {0, cos(angle), -sin(angle)},
                                     {0, sin(angle), cos(angle)}};
                return {3, 3, CV_64FC1, data};
            }
            case yAxis:{
                yAngle += angle;
                double data[3][3] = {{cos(angle) ,0 , sin(angle)},
                                     {0, 1, 0},
                                     {-sin(angle), 0, cos(angle)}};
                return {3, 3, CV_64FC1, data};
            }
            case zAxis:{
                zAngle += angle;
                double data[3][3] = {{cos(angle) ,-sin(angle) , 0},
                                     {sin(angle), cos(angle), 0},
                                     {0, 0, 1}};
                return {3, 3, CV_64FC1, data};
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

    cv::Mat coordinateSystem::getRotationMatrix() {
        return rotationMatrix;
    }
}