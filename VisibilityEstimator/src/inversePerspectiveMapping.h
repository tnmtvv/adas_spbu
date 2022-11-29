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
        inversePerspectiveMapping(int sizeX, int sizeY, int fx, int fy,
            int opticalCenterX, int opticalCenterY, int cameraHeight, int pitch) :
            sizeX(sizeX), sizeY(sizeY), focalLengthX(fx), focalLengthY(fy),
            opticalCenterX(opticalCenterX), opticalCenterY(opticalCenterY), cameraHeight(cameraHeight), pitch(CV_PI* pitch / 180)
        {
        }

        cv::Mat inversePerspectiveMap(cv::Mat frame, int horizontalLineCoordinate);

    private:
        int sizeX;
        int sizeY;

        int focalLengthX;
        int focalLengthY;

        int opticalCenterX;
        int opticalCenterY;

        int cameraHeight;
        double pitch;

        cv::Mat getProjectiveTransfromationMatrix(cv::Size size);

        cv::Mat getMatrixWithoutSecondColumn(cv::Mat matrix);
    };
}

#endif