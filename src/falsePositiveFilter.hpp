#pragma once
#ifndef FALSEPOSITIVEFILTER_HPP
#define FALSEPOSITIVEFILTER_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "openpose/headers.hpp"

namespace PedestrianFilter
{
    class falsePositiveFilter
    {
    public:
        static void filterizeFrame(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr, cv::Mat &image);

    private:
        static const float calculatePersonHeight(float **data, const std::array<cv::Point, 2> points);

        static void applyFilter(const bool isValid, cv::Mat &image, const std::array<cv::Point, 2> points);

        static const std::array<cv::Point, 2> findBoudaryPoints(float **poseKeypoints, const int count);
    };
}

#endif