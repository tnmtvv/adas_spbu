//
// Created by eeuri on 13.11.2022.
//
#include "coordinateSystem.h"

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
                initializateMatrix(0, 0, 1, matrix);
                initializateMatrix(1, 1, cos(angle), matrix);
                initializateMatrix(1, 2, -sin(angle), matrix);
                initializateMatrix(2, 1, sin(angle), matrix);
                initializateMatrix(2, 2, cos(angle), matrix);
                break;
            }
            case yAxis:{
                yAngle += angle;
                initializateMatrix(0, 0, cos(angle), matrix);
                initializateMatrix(0, 2, sin(angle), matrix);
                initializateMatrix(1, 1, 1, matrix);
                initializateMatrix(2, 0, -sin(angle), matrix);
                initializateMatrix(2, 2, cos(angle), matrix);
                break;
            }
            case zAxis:{
                zAngle += angle;
                initializateMatrix(0, 0, cos(angle), matrix);
                initializateMatrix(0, 1, -sin(angle), matrix);
                initializateMatrix(1, 0, sin(angle), matrix);
                initializateMatrix(1, 1, cos(angle), matrix);
                initializateMatrix(2, 2, 1, matrix);
                break;
            }
        }

        return matrix;
    }

    void coordinateSystem::initializateMatrix(int i, int j, double value, cv::Mat &matrix) {
        matrix.at<double>(i, j) = value;
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

    void coordinateSystem::setCoordinatesOfCenter(cv::Vec3d vector) {
        coordinatesOfCenterOfCoordinateSystem.at<double>(0, 0) = vector[0];
        coordinatesOfCenterOfCoordinateSystem.at<double>(1, 0) = vector[1];
        coordinatesOfCenterOfCoordinateSystem.at<double>(2, 0) = vector[2];
    }
} // coordinateSystem