#pragma once
#ifndef CONTOURDETECTOR_H
#define CONTOURDETECTOR_H

#include "opencv2/opencv.hpp"

namespace EdgeDetector
{
    class contourDetector
    {
    public:
        static std::vector<std::vector<cv::Point>> detectContours(cv::Mat image);
    };
}

#endif