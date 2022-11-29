#include "imageFilter.h"

namespace EdgeDetector
{
    cv::Mat imageFilter::applyCannyOperator(cv::Mat frame)
    {
        cv::Mat grayImage, blurGrayImage, resultImage;

        //cvtColor(_frame, grayImage, COLOR_BGR2GRAY);
        GaussianBlur(frame, blurGrayImage, cv::Size(5, 5), 1.4, 1.4, 1);
        Canny(blurGrayImage, resultImage, 20, 30);

        return resultImage;
    }

    cv::Mat imageFilter::selectMinimumForEachPixel(std::vector<cv::Mat> images)
    {
        cv::Mat average = cv::Mat::zeros(images[0].size(), CV_8UC1);

        for (int row = 0; row < images[0].rows; row++)
        {
            for (int col = 0; col < images[0].cols; col++)
            {
                uchar value = images[0].at<uchar>(row, col);

                for (int imageNumber = 1; imageNumber < images.size(); imageNumber++)
                {
                    uchar newValue = images[imageNumber].at<uchar>(row, col);

                    if (newValue < value)
                    {
                        value = newValue;
                    }
                }

                average.at<uchar>(row, col) = value;
            }
        }

        return average;
    }
}