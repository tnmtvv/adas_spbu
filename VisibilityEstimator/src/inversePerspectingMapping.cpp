#include "inversePerspectiveMapping.h"

#include <filesystem>

namespace distanceEstimator
{
    cv::Mat inversePerspectiveMapping::inversePerspectiveMap(cv::Mat frame, cv::Point vanishingPoint, cv::Point farthestVisiblePoint)
    {
        cv::Mat warpedImage = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), frame.type());
        cv::Rect croppedRectangle = cv::Rect(0, vanishingPoint.y, frame.cols, frame.rows - vanishingPoint.y);
        cv::Mat croppedImage = frame(croppedRectangle);

        const float pitch = -atan(((float)vanishingPoint.y - (float)sizeY / 2) / (float)focalLength);

        cv::Mat projectiveTransformationMatrix = getProjectiveTransformationMatrix(croppedImage.size(), pitch);
        cv::warpPerspective(croppedImage, warpedImage, projectiveTransformationMatrix,
                            cv::Size(warpedImage.cols, warpedImage.rows), cv::WARP_INVERSE_MAP);

        //cv::resize(warpedImage, warpedImage, cv::Size(frame.cols, frame.rows + 300));

        return warpedImage;
    }

    cv::Mat inversePerspectiveMapping::getProjectiveTransformationMatrix(cv::Size size, float pitch)
    {
        cv::Mat projectionMatrix = (cv::Mat_<float>(4, 3) <<
                1, 0, -size.width / 2,
                0, 1, -size.height / 2,
                0, 0, 0,
                0, 0, 1);

        cv::Mat rotationMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, cos(pitch), -sin(pitch), 0,
                0, sin(pitch), cos(pitch), 0,
                0, 0, 0, 1);

        cv::Mat translationMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, -cameraHeight / sin(pitch),
                0, 0, 0, 1);

        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 4) <<
                focalLength, 0, size.width / 2, 0,
                0, focalLength, size.height / 2, 0,
                0, 0, 1, 0);

        return cameraMatrix * translationMatrix * rotationMatrix * projectionMatrix;
    }

    cv::Mat inversePerspectiveMapping::getMatrixWithoutSecondColumn(cv::Mat matrix)
    {
        return (cv::Mat_<float>(3, 3) <<
                                      matrix.at<float>(0, 0), matrix.at<float>(0, 2), matrix.at<float>(0, 3),
                matrix.at<float>(1, 0), matrix.at<float>(1, 2), matrix.at<float>(1, 3),
                matrix.at<float>(2, 0), matrix.at<float>(2, 2), matrix.at<float>(2, 3));
    }
}