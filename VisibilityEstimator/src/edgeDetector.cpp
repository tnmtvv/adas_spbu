#include "EdgeDetector.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "contoursOrdering.h"

#include <filesystem>
#include <algorithm>
#include <map>
#include <set>

namespace EdgeDetector
{
    //Mat EdgeDetector::detectStraightLines(Mat frame)
    //{
    //    Mat grayFrame, dst;
    //
    //    dst = frame.clone();
    //
    //    if (frame.channels() != 1)
    //    {
    //        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
    //    }
    //    else
    //    {
    //        grayFrame = frame.clone();
    //    }
    //    //vector<Vec4i> lines;
    //    //HoughLinesP(grayFrame, lines, 1 , CV_PI / 180, 100, 100, 10);
    //
    //    //for (size_t i = 0; i < lines.size(); i++)
    //    //{
    //    //    Vec4i l = lines[i];
    //    //    line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
    //    //}
    //    vector<Vec2f> lines;
    //    HoughLines(grayFrame, lines, 1, CV_PI / 180, 180, 0, 0);
    //    for (size_t i = 0; i < lines.size(); i++)
    //    {
    //        float rho = lines[i][0], theta = lines[i][1];
    //        Point pt1, pt2;
    //        double a = cos(theta), b = sin(theta);
    //        double x0 = a * rho, y0 = b * rho;
    //        int c = (y0 - 720) / a;
    //        pt1.x = cvRound(x0 + c * (-b));
    //        pt1.y = cvRound(y0 + c * (a));
    //        pt2.x = cvRound(x0 - c * (-b));
    //        pt2.y = cvRound(y0 - c * (a));
    //        line(dst, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
    //    }
    //
    //    return dst;
    //}

    cv::Point edgeDetector::findFarthestVisiblePoint(std::vector<std::vector<cv::Point>> contours, cv::Point vanishingPoint, double* contourAngle)
    {
        std::vector<int> sortedContoursIndices = contoursOrdering::sortContoursByTheirSizesDescending(contours);
        int numberOfConsideredContours = contours.size() >= 10 ? 10 : contours.size();
        cv::Point farthestVisiblePoint = cv::Point(INT_MAX, INT_MAX);

        cv::Point testi123;

        for (int i = 0; i < numberOfConsideredContours; i++)
        {
            auto pointWithMaxY = contoursOrdering::findPointWithMaxY(contours[sortedContoursIndices[i]]);
            auto coefficient = findCoefficient(vanishingPoint, pointWithMaxY);

            if (coefficient * pointWithMaxY.y < farthestVisiblePoint.y && pointWithMaxY.y > vanishingPoint.y)
            {
                farthestVisiblePoint = cv::Point(pointWithMaxY.x, (int)(coefficient * pointWithMaxY.y));
                testi123 = pointWithMaxY;
            }
        }

        //auto coefficient = findCoefficient(vanishingPoint, contours[testi123]);
        *contourAngle = countContourCos(vanishingPoint, testi123);

        if (farthestVisiblePoint.y == 245)
        {
            int a = 10;
        }
        return farthestVisiblePoint;
    }

    double edgeDetector::countContourCos(cv::Point vanishingPoint, cv::Point contourPoint)
    {
        double hypotenuse = sqrt(pow(contourPoint.x - vanishingPoint.x, 2) + pow(contourPoint.y - vanishingPoint.y, 2));
        double cathetus = contourPoint.y - vanishingPoint.y;
        double cosine = cathetus / hypotenuse;

        return cosine;
    }

    double edgeDetector::findCoefficient(cv::Point vanishingPoint, cv::Point point)
    {
        double cosine = countContourCos(vanishingPoint, point);
        double angle = 180 * acos(cosine) / CV_PI;
        if (angle > 70)
        {
            return 1.064;
        }
        return 1;
    }
}