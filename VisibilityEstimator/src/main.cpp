#include "opencv2/highgui.hpp"
#include "edgeDetector.h"
#include "inversePerspectiveMapping.h"
#include "contourDrawer.h"
#include "contoursMerger.h"
#include "imageFilter.h"
#include "contourDetector.h"
#include "edgeDetector.h"
#include <iostream>

void showFilteredVideo(std::string path);

int main(int argc, char** argv)
{
    showFilteredVideo("../fog1.mp4");
    return 0;
}

void showFilteredVideo(std::string path)
{
    cv::VideoCapture video(path);

    if (!video.isOpened())
    {
        return;
    }

    cv::namedWindow("Average", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Frame with horizontal line", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Connected contours", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Warped image", cv::WINDOW_AUTOSIZE);

    bool empty = false;
    int counter = 0;
    std::vector<cv::Mat> framesPackage;

    while (!empty)
    {
        for (int i = 0; i < 4; i++)
        {
            cv::Mat frame;
            cv::Mat grayFrame;
            video >> frame;
            if (frame.empty())
            {
                empty = true;
                break;
            }

            cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
            framesPackage.push_back(grayFrame);
        }

        if (framesPackage.size() == 0)
        {
            break;
        }

        // averaging
        cv::Mat averageImage = EdgeDetector::imageFilter::selectMinimumForEachPixel(framesPackage);

        // detecting and merging contours
        auto mergedContours = EdgeDetector::contourDetector::detectContours(averageImage);
        cv::Mat dst2 = cv::Mat::zeros(averageImage.rows, averageImage.cols, CV_8UC3);
        EdgeDetector::contourDrawer::colorContours(dst2, mergedContours);
        
        // drawing horizontal line
        cv::Mat frameWithLine;
        cvtColor(averageImage, frameWithLine, cv::COLOR_GRAY2BGR);

        cv::Point vanishingPoint = cv::Point(324, 190);
        //cv::Point vanishingPoint = cv::Point(650, 412);
        double angle = 0;
        auto farthestVisiblePoint = EdgeDetector::edgeDetector::findFarthestVisiblePoint(mergedContours, vanishingPoint, &angle);
        EdgeDetector::contourDrawer::drawHorizontalLine(frameWithLine, farthestVisiblePoint);

        std::cout << 180 * acos(angle) / CV_PI;
        std::cout << farthestVisiblePoint;
        std::cout << '\n';
        
        // IPM
        //const int sizeX = 1280; // size of frame
        //const int sizeY = 720;
        const int sizeX = 640; // size of frame
        const int sizeY = 360;
        const int focalLength = 500;
        const int opticalCenterX = sizeX / 2;
        const int opticalCenterY = sizeY / 2;
        const double cameraHeight = 1.3f;
        const float pitch = atan(((float)vanishingPoint.y - (float)sizeY / 2) / (float)focalLength);

        distanceEstimator::inversePerspectiveMapping ipm(sizeX, sizeY, focalLength, opticalCenterX, opticalCenterY, cameraHeight, pitch);
        cv::Mat warpedImage = ipm.inversePerspectiveMap(frameWithLine, farthestVisiblePoint.y);

        cv::imshow("Warped image", warpedImage);
        cv::imshow("Average", averageImage);
        cv::imshow("Frame with horizontal line", frameWithLine);
        cv::imshow("Connected contours", dst2);
        framesPackage.clear();

        counter++;
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    video.release();
}