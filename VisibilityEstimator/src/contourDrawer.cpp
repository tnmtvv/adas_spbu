#include "contourDrawer.h"
#include "opencv2/imgproc.hpp"

namespace EdgeDetector
{
    void contourDrawer::colorContours(cv::Mat frame, std::vector<std::vector<cv::Point>> contours)
    {
        for (int idx = 0; idx < contours.size(); idx++)
        {
            cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
            drawContours(frame, contours, idx, color, 1, 8);
        }
    }

    void contourDrawer::drawHorizontalLine(cv::Mat frame, cv::Point point)
    {
        cv::Point p1(0, point.y);
        cv::Point p2(frame.cols, point.y);
        int thickness = 1;

        line(frame, p1, p2, cv::Scalar(0, 0, 255), thickness, cv::LINE_4);
    }
}