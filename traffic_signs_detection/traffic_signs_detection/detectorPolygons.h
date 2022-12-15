#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class detectorPolygons
{

public:

    static std::vector<cv::Vec3f> detectCircle(cv::Mat frame);
};
