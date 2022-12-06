#include "inversePerspectiveMapping.h"

#include <filesystem>

namespace distanceEstimator
{
    cv::Mat inversePerspectiveMapping::inversePerspectiveMap(cv::Mat frame, int horizontalLineCoordinate)
    {
        cv::Mat warpedImage;
        cv::Rect croppedRectangle = cv::Rect(0, horizontalLineCoordinate, frame.cols, frame.rows - horizontalLineCoordinate);
        cv::Mat croppedImage = frame(croppedRectangle);

        cv::Mat projectiveTransformationMatrix = getProjectiveTransfromationMatrix(croppedImage.size());
        cv::warpPerspective(croppedImage, warpedImage, projectiveTransformationMatrix,
                            cv::Size(frame.cols, frame.rows), cv::WARP_INVERSE_MAP);

        cv::resize(warpedImage, warpedImage, cv::Size(frame.cols, frame.rows));

        return warpedImage;
    }

    cv::Mat inversePerspectiveMapping::getProjectiveTransfromationMatrix(cv::Size size)
    {
        cv::Mat rotationMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, 0,
                0, cos(pitch), -sin(pitch), 0,
                0, sin(pitch), cos(pitch), 0,
                0, 0, 0, 1);

        cv::Mat translationMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, 0, -size.width / 2,
                0, 1, 0, 0,
                0, 0, 1, -cameraHeight / sin(pitch),
                0, 0, 0, 1);

        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 4) <<
                focalLength, 0, size.width / 2, 0,
                0, focalLength, size.height / 2, 0,
                0, 0, 1, 0);

        return getMatrixWithoutSecondColumn(cameraMatrix * translationMatrix * rotationMatrix);
    }

    cv::Mat inversePerspectiveMapping::getMatrixWithoutSecondColumn(cv::Mat matrix)
    {
        return (cv::Mat_<float>(3, 3) <<
                matrix.at<float>(0, 0), matrix.at<float>(0, 2), matrix.at<float>(0, 3),
                matrix.at<float>(1, 0), matrix.at<float>(1, 2), matrix.at<float>(1, 3),
                matrix.at<float>(2, 0), matrix.at<float>(2, 2), matrix.at<float>(2, 3));
    }
}