#pragma once
#ifndef EDGEDETECTOR_H
#define EDGEDETECTOR_H

#include "opencv2/imgproc.hpp"

#include <string>
#include <set>

namespace EdgeDetector
{
    class edgeDetector
    {
    public:
        //static cv::Mat findComponents(cv::Mat frame);

        //static cv::Mat detectStraightLines(cv::Mat frame);

        static cv::Point findFarthestVisiblePoint(std::vector<std::vector<cv::Point>> contours, cv::Point vanishingPoint, double* contourAngle);

    private:

        static double countContourCos(cv::Point vanishingPoint, cv::Point contourPoint);

        static double findCoefficient(cv::Point vanishingPoint, cv::Point contour);
    };
}

#endif