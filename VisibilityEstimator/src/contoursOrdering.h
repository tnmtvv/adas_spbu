#pragma once
#ifndef CONTOURSORDERING_H
#define CONTOURSORDERING_H

#include <opencv2/opencv.hpp>

namespace EdgeDetector
{
    class contoursOrdering
    {
    public:
        static cv::Point findPointWithMaxY(std::vector<cv::Point>);

        static std::vector<int> sortContoursByTheirSizesDescending(std::vector<std::vector<cv::Point>> contours);
    };
}

#endif