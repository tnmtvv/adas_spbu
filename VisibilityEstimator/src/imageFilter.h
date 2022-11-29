#pragma once
#ifndef IMAGEFILTER_H
#define IMAGEFILTER_H

#include "opencv2/imgproc.hpp"

namespace EdgeDetector
{
    class imageFilter
    {
    public:
        static cv::Mat applyCannyOperator(cv::Mat frame);

        static cv::Mat selectMinimumForEachPixel(std::vector<cv::Mat> images);
    };
}

#endif