#pragma once
#ifndef INVERSEPERSPECTIVEMAPPING_H
#define INVERSEPERSPECTIVEMAPPING_H

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <string>
#include <vector>

namespace distanceEstimator
{
    class inversePerspectiveMapping
    {
    public:
        inversePerspectiveMapping(int sizeX, int sizeY, int focalLength, double cameraHeight) :
                sizeX(sizeX), sizeY(sizeY), focalLength(focalLength), cameraHeight(cameraHeight)
        {
        }

        cv::Mat inversePerspectiveMap(cv::Mat frame, cv::Point vanishingPoint, cv::Point farthestVisiblePoint);

    private:
        int sizeX;
        int sizeY;

        double cameraHeight;
        int focalLength;

        cv::Mat getProjectiveTransformationMatrix(cv::Size size, float pitch);

        cv::Mat getMatrixWithoutSecondColumn(cv::Mat matrix);
    };
}

#endif