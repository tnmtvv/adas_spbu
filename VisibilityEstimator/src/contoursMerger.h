#pragma once
#ifndef CONTOURSMERGER_H
#define CONTOURSMERGER_H

#include "opencv2/imgproc.hpp"

#include <string>
#include <set>

namespace EdgeDetector
{
    class contoursMerger
    {
    public:
        static std::vector<std::vector<cv::Point>> connectContours(std::vector<std::vector<cv::Point>> contours,
            int imageSizeX, int imageSizeY);

        static std::set<int> findNearestContours(std::vector<std::vector<cv::Point>> contours, int mainContourIndex,
            std::vector<std::vector<int>> contoursIndexesMatrix, cv::Point point);

        static std::vector<std::vector<cv::Point>> mergeContours(std::vector<std::vector<cv::Point>> contours,
            std::vector<std::vector<int>> indexMatrix, int mainContourIndex, std::vector<cv::Point> mainContour);

    private:
        static std::vector<std::vector<cv::Point>> removeZeroSizeContours(std::vector<std::vector<cv::Point>> contours);
    };
}
#endif