//
// Created by eeuri on 13.11.2022.
//

#ifndef UNTITLED_COORDINATESYSTEM_H
#define UNTITLED_COORDINATESYSTEM_H

#include "opencv2/opencv.hpp"

enum Axis{
    xAxis,
    yAxis,
    zAxis
};

namespace coordinateSystem {
    class coordinateSystem {
    private:
        cv::Mat coordinatesOfCenterOfCoordinateSystem;
        cv::Mat rotationMatrix;
        cv::Mat generateRotationMatrix(double angle, Axis axis);
        static void initializateMatrix(int i, int j, double value, cv::Mat &matrix);
        double xAngle;
        double yAngle;
        double zAngle;

    public:
        void move(const cv::Vec3d& offset);
        coordinateSystem();
        void rotate(double angle, Axis axis);
        cv::Mat moveToGlobalCoordinates(const cv::Vec3d& coordinates);
        cv::Mat moveToLocalCoordinates(const cv::Vec3d& coordinates);
        [[nodiscard]] double getAngle(Axis axis) const;
        cv::Mat getCoordinatesOfCenter();
        void setCoordinatesOfCenter(cv::Vec3d vector);
    };
}
#endif //UNTITLED_COORDINATESYSTEM_H
