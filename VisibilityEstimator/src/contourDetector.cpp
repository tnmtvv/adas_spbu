#include "contourDetector.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "imageFilter.h"
#include "contoursMerger.h"

namespace EdgeDetector
{
    //cv::Mat contoursDetector::findComponents(cv::Mat frame)
    //{
    //    //Mat src = applyLaplacianOperator();
    //    cv::Mat src = applyCannyOperator(frame);
    //    cv::Mat labels;
    //    int numberOfComponents = connectedComponents(src, labels);

    //    std::vector<cv::Vec3b> colors(numberOfComponents);
    //    colors[0] = cv::Vec3b(0, 0, 0);

    //    for (int i = 0; i < numberOfComponents; i++)
    //    {
    //        colors[i] = cv::Vec3b((50 + (rand() % 150)) & 255, (50 + (rand() % 100)) & 255, (50 + (rand() % 100)) & 255);
    //    }

    //    cv::Mat out = cv::Mat::zeros(src.size(), CV_8UC3);

    //    for (int i = 0; i < src.cols; i++)
    //    {
    //        for (int j = 0; j < src.rows; j++)
    //        {
    //            if (labels.at<int>(cv::Point(i, j)) != 0)
    //            {
    //                out.at<cv::Vec3b>(cv::Point(i, j)) = colors[(int)labels.at<int>(cv::Point(i, j))];
    //            }
    //        }
    //    }

    //    return out;
    //}

    std::vector<std::vector<cv::Point>> contourDetector::detectContours(cv::Mat image)
    {
        std::vector<std::vector<cv::Point>> contours;

        cv::Mat cannyImage = imageFilter::applyCannyOperator(image);
        findContours(cannyImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        if (contours.size() == 0)
        {
            return contours;
        }

        auto mergedContours = contoursMerger::connectContours(contours, image.cols, image.rows);

        contours.clear();

        return mergedContours;
    }
}