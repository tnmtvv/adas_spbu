#include <iostream>

#include <algorithm>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

//sample: ./adas_spbu video_name.mp4

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        return 1;
    }
    cv::VideoCapture newFrameCap(argv[1]);
    if (!newFrameCap.isOpened())
    {
        return 1;
    }
    int frame_width = newFrameCap.get(cv::CAP_PROP_FRAME_WIDTH);
    //обрезка капота
    int cut_height = 300;
    int frame_height = newFrameCap.get(cv::CAP_PROP_FRAME_HEIGHT) - cut_height;
    double roll = -10.0;


    bool empty = false;
    while (!empty) {
        cv::Mat frame;
        newFrameCap >> frame;
        empty = frame.rows == 0;
        if (empty)
        {
            break;
        }
        cv::Point2f center(frame.cols / 2.0, frame.rows / 2.0);
        cv::Mat rotation_matix = cv::getRotationMatrix2D(center, roll, 1.0);
        cv::Mat rotated_image(frame.rows, frame.cols, CV_8UC3, cv::Scalar(0));
        //убираем ролл
        cv::warpAffine(frame, rotated_image, rotation_matix, frame.size());
        rotated_image = rotated_image(cv::Rect(0, 0, frame_width, frame_height));

        cv::imshow("frame", rotated_image);
        cv::waitKey(1);
    }
    newFrameCap.release();
    return 0;
}
