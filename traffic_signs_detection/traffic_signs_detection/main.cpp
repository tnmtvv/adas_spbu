#include "imageSegmentation.h"
#include "main.h"
#include "detectorPolygons.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


int main()
{
   std::string image_path = cv::samples::findFile("autosave01_02_2012_09_18_35.jpg");

    cv::Mat frame = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::Mat red = imageSegmentation::highlightRed(frame);
    cv::Mat white = imageSegmentation::highlightWhite(frame);
    cv::Mat blue = imageSegmentation::highlightBlue(frame);
    cv::Mat yellow = imageSegmentation::highlightYellow(frame);
    std::vector<cv::Vec3f> circlesRed = detectorPolygons::detectCircle(red);
    std::vector<cv::Vec3f> circlesBlue = detectorPolygons::detectCircle(blue);
    std::vector<cv::Vec3f> circlesYellow = detectorPolygons::detectCircle(yellow);
    std::vector<cv::Vec3f> circlesWhite = detectorPolygons::detectCircle(white);

    for (size_t i = 0; i < circlesRed.size(); i++)
    {
        cv::Vec3i coordinates = circlesRed[i];
        int radius = coordinates[2];
        cv::Point upperVertex = cv::Point(coordinates[0] - radius - 0.5, coordinates[1] + radius + 0.5);
        cv::Point lowerVertex = cv::Point(coordinates[0] + radius + 0.5, coordinates[1] - radius - 0.5);
        cv::rectangle(red, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::rectangle(frame, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
    }

    for (size_t i = 0; i < circlesBlue.size(); i++)
    {
        cv::Vec3i coordinates = circlesBlue[i];
        int radius = coordinates[2];
        cv::Point upperVertex = cv::Point(coordinates[0] - radius - 0.5, coordinates[1] + radius + 0.5);
        cv::Point lowerVertex = cv::Point(coordinates[0] + radius + 0.5, coordinates[1] - radius - 0.5);
        cv::rectangle(blue, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::rectangle(frame, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
    }

    for (size_t i = 0; i < circlesYellow.size(); i++)
    {
        cv::Vec3i coordinates = circlesYellow[i];
        int radius = coordinates[2];
        cv::Point upperVertex = cv::Point(coordinates[0] - radius - 0.5, coordinates[1] + radius + 0.5);
        cv::Point lowerVertex = cv::Point(coordinates[0] + radius + 0.5, coordinates[1] - radius - 0.5);
        cv::rectangle(yellow, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::rectangle(frame, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
    }

    for (size_t i = 0; i < circlesWhite.size(); i++)
    {
        cv::Vec3i coordinates = circlesWhite[i];
        int radius = coordinates[2];
        cv::Point upperVertex = cv::Point(coordinates[0] - radius - 0.5, coordinates[1] + radius + 0.5);
        cv::Point lowerVertex = cv::Point(coordinates[0] + radius + 0.5, coordinates[1] - radius - 0.5);
        cv::rectangle(white, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::rectangle(frame, upperVertex, lowerVertex, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
    }
    cv::imshow("red", red);
    cv::imshow("white", white);
    cv::imshow("blue", blue);
    cv::imshow("yellow", yellow);
    cv::imshow("detected circles", frame);
    cv::waitKey(0);

  /*  cv::VideoCapture video("03300026_7777.MP4");
    if (!video.isOpened())
    {
        return 1;
    }
    cv::Mat redImage;
    cv::Mat blueImage;
    cv::Mat yellowImage;
    cv::Mat whiteImage;
    while (video.isOpened())
    {
        cv::Mat frame;
        video >> frame;
        cv::Mat fr;
        cv::Mat red = highlightRed(frame, redImage);
        cv::Mat white = highlightWhite(frame, whiteImage);
        cv::Mat blue = highlightBlue(frame, blueImage);
        cv::Mat yellow = highlightYellow(frame, yellowImage);
        cv::imshow("red", red);
        cv::imshow("white", white);
        cv::imshow("blue", blue);
        cv::imshow("yellow", yellow);
        cv::waitKey(0);
    } 
    video.release();
    */
 /**   std::vector<cv::Vec3f> circles;
    HoughCircles(redImage, circles, cv::HOUGH_GRADIENT, 1,
        redImage.rows / 16,  // change this value to detect circles with different distances to each other
        100, 30, 1, 200 // change the last two parameters
   // (min_radius & max_radius) to detect larger circles
    );
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle(redImage, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle(img, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    cv::imshow("detected circles", img);
    cv::waitKey();
    return EXIT_SUCCESS;
    */
    //cv::waitKey(0); // Wait for a keystroke in the window
   /* cv::Mat src = cv::imread(cv::samples::findFile("2035.jpg"), cv::IMREAD_COLOR);
    // Check if image is loaded fine
    if (src.empty()) {
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", "2035.jpg");
        return EXIT_FAILURE;
    }
    cv::Mat gray;
    cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
        gray.rows / 16,  // change this value to detect circles with different distances to each other
        100, 30, 30, 60 // change the last two parameters
   // (min_radius & max_radius) to detect larger circles
    );
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle(src, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle(src, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    imshow("detected circles", src);
    cv::waitKey();
    return EXIT_SUCCESS;
}
    */
   
}
