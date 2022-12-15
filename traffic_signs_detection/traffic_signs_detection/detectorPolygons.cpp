#include <iostream>
#include "detectorPolygons.h"


std::vector<cv::Vec3f> detectorPolygons::detectCircle(cv::Mat frame)
{
    cv::Mat grayImage;
    cvtColor(frame, grayImage, cv::COLOR_BGR2GRAY);
    medianBlur(grayImage, grayImage, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1,
        grayImage.rows / 16,  // change this value to detect circles with different distances to each other
        100, 30, 1, 40); // change the last two parameters
       // (min_radius & max_radius) to detect larger circles
    return circles;
 }