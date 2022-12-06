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
        inversePerspectiveMapping(int sizeX, int sizeY, int focalLength,
                                  int opticalCenterX, int opticalCenterY, double cameraHeight, float pitch) :
                sizeX(sizeX), sizeY(sizeY), focalLength(focalLength),
                opticalCenterX(opticalCenterX), opticalCenterY(opticalCenterY), cameraHeight(cameraHeight), pitch(pitch)
        {
        }

        cv::Mat inversePerspectiveMap(cv::Mat frame, int horizontalLineCoordinate);

    private:
        int sizeX;
        int sizeY;

        int focalLength;

        int opticalCenterX;
        int opticalCenterY;

        double cameraHeight;
        float pitch;

        cv::Mat getProjectiveTransfromationMatrix(cv::Size size);

        cv::Mat getMatrixWithoutSecondColumn(cv::Mat matrix);
    };
}

#endif