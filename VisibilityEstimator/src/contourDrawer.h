#pragma once
#ifndef CONTOURDRAWER_H
#define CONTOURDRAWER_H

#include "opencv2/opencv.hpp"

namespace EdgeDetector
{
    class contourDrawer
    {
    public:
        static void colorContours(cv::Mat frame, std::vector<std::vector<cv::Point>> contours);

        static void drawHorizontalLine(cv::Mat frame, cv::Point point);
    };
}

#endif