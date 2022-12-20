#include <iostream>
#include <algorithm>
#include "src/falsePositiveFilter.hpp"

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

    // Configuring OpenPose
    op::opLog("Configuring OpenPose...", op::Priority::High);
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

    // Starting OpenPose
    op::opLog("Starting thread(s)...", op::Priority::High);
    opWrapper.start();

    bool empty = false;
    while (!empty)
    {
        cv::Mat frame;
        newFrameCap >> frame;
        empty = frame.rows == 0;
        if (empty)
        {
            break;
        }

        op::Matrix imageToProcess = OP_CV2OPCONSTMAT(frame);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);

        if (datumProcessed != nullptr && !datumProcessed->empty())
        {

            cv::Mat cvMat = OP_OP2CVCONSTMAT(datumProcessed->at(0)->cvOutputData);
            PedestrianFilter::falsePositiveFilter::filterizeFrame(datumProcessed, cvMat);
            if (!cvMat.empty())
            {
                cv::imshow("frame", cvMat);
                cv::waitKey(1);
            }
        }
        else
        {
            op::opLog("Image could not be processed.", op::Priority::High);
        }
    }

    newFrameCap.release();
    return 0;
}
