#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class imageSegmentation
{
public:
    static cv::Mat highlightRed(cv::Mat originalImage);

    static cv::Mat highlightYellow(cv::Mat originalImage);

    static cv::Mat highlightBlue(cv::Mat originalImage);

    static cv::Mat highlightWhite(cv::Mat originalImage);
};
