#include "falsePositiveFilter.hpp"

namespace PedestrianFilter
{

    const float falsePositiveFilter::calculatePersonHeight(float **data, const std::array<cv::Point, 2> points)
    {
        const int minY = points[0].y;
        const float hCam = 1.3f;
        const int focalDistance = 750;
        const int hor = 425;
        const float humanDistance = std::abs(hCam * focalDistance / (minY - hor));
        const int size = std::max(points[1].x - points[0].x, points[1].y - points[0].y);
        const float humanHeight = humanDistance * size / focalDistance;
        return humanHeight;
    }

    void falsePositiveFilter::applyFilter(const bool isValid, cv::Mat &image, const std::array<cv::Point, 2> points)
    {
        cv::Scalar color;
        if (isValid)
        {
            color = cv::Scalar(0, 255, 0);
        }
        else
        {
            color = cv::Scalar(0, 0, 255);
        }

        cv::rectangle(image, points[0], points[1], color);
    }

    const std::array<cv::Point, 2> falsePositiveFilter::findBoudaryPoints(float **poseKeypoints, const int size)
    {
        std::array<cv::Point, 2> points;

        int minX = poseKeypoints[0][0];
        int minY = poseKeypoints[0][1];
        int maxX = poseKeypoints[0][0];
        int maxY = poseKeypoints[0][1];

        for (auto bodyPart = 1; bodyPart < size; bodyPart++)
        {
            if (poseKeypoints[bodyPart][0] > maxX)
            {
                maxX = poseKeypoints[bodyPart][0];
            }

            if (poseKeypoints[bodyPart][0] < minX)
            {
                minX = poseKeypoints[bodyPart][0];
            }

            if (poseKeypoints[bodyPart][1] > maxY)
            {
                maxY = poseKeypoints[bodyPart][1];
            }

            if (poseKeypoints[bodyPart][1] < minY)
            {
                minY = poseKeypoints[bodyPart][1];
            }
        }

        points[0] = cv::Point(minX, minY);
        points[1] = cv::Point(maxX, maxY);

        return points;
    }

    void falsePositiveFilter::filterizeFrame(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr, cv::Mat &image)
    {
        try
        {
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                const auto &keypoints = datumsPtr->at(0)->poseKeypoints;
                for (auto person = 0; person < keypoints.getSize(0); person++)
                {
                    float **poseKeypoints = new float *[keypoints.getSize(1)];
                    int i = 0;

                    for (auto bodyPart = 0; bodyPart < keypoints.getSize(1); bodyPart++, i++)
                    {
                        if (keypoints[{person, bodyPart, 0}] == 0 && keypoints[{person, bodyPart, 1}] == 0)
                        {
                            i--;
                        }
                        else
                        {
                            poseKeypoints[i] = new float[2];
                            poseKeypoints[i][0] = keypoints[{person, bodyPart, 0}];
                            poseKeypoints[i][1] = keypoints[{person, bodyPart, 1}];
                        }
                    }

                    const int size = i;
                    const std::array<cv::Point, 2> points = findBoudaryPoints(poseKeypoints, size);
                    const float height = calculatePersonHeight(poseKeypoints, points);

                    applyFilter(height < 2 && height > 0.8, image, points);

                    for (auto bodyPart = 0; bodyPart < size; bodyPart++)
                    {
                        delete[] poseKeypoints[bodyPart];
                    }
                    delete[] poseKeypoints;
                }
            }
            else
            {
                op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
            }
        }
        catch (const std::exception &e)
        {
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}