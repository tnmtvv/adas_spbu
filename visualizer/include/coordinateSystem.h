#ifndef UNTITLED_COORDINATESYSTEM_H
#define UNTITLED_COORDINATESYSTEM_H

#include "opencv2/opencv.hpp"

namespace coordinateSystem {
    enum Axis{
        xAxis,
        yAxis,
        zAxis
    };

    class coordinateSystem {
    private:
        cv::Mat coordinatesOfCenterOfCoordinateSystem;
        cv::Mat rotationMatrix;
        cv::Mat generateRotationMatrix(double angle, Axis axis);
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
        void setCoordinatesOfCenter(const cv::Vec3d& vector);

        [[maybe_unused]] cv::Mat getRotationMatrix();
    };
}

#endif
