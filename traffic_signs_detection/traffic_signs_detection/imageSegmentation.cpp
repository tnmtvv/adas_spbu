#include "imageSegmentation.h"
#include <iostream>

cv::Mat imageSegmentation::highlightRed(cv::Mat originalImage)
{
	cv::Mat imageHSV;
	cv::Mat redImage;
    cv::medianBlur(originalImage, originalImage, 1);
	cv::cvtColor(originalImage, imageHSV, cv::COLOR_BGR2HSV);
	cv::Mat redImage1, redImage2;
    //cv::inRange(imageHSV, cv::Scalar(120, 0, 0), cv::Scalar(170, 255, 255), blueImage);
	cv::inRange(imageHSV, cv::Scalar(0, 100, 20), cv::Scalar(10, 255, 255), redImage1);
	cv::inRange(imageHSV, cv::Scalar(160, 100, 20), cv::Scalar(179, 255, 255), redImage2);
	redImage = redImage1 + redImage2;
	//cv::inRange(imageHSV, cv::Scalar(90, 50, 70), cv::Scalar(128, 255, 255), blueImage);
	//cv::inRange(imageHSV, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), yellowImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 100), cv::Scalar(180, 18, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 240), cv::Scalar(255, 15, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 150), cv::Scalar(360, 60, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(190, 130, 50), cv::Scalar(236, 255, 255), blueImage);
	//cv::inRange(imageHSV, cv::Scalar(324, 130, 50), cv::Scalar(360, 255, 255), redImage1);
//	cv::inRange(imageHSV, cv::Scalar(0, 130, 50), cv::Scalar(30, 255, 255), redImage2);
//	redImage = redImage1 + redImage2;
	//cv::inRange(imageHSV, cv::Scalar(90, 50, 70), cv::Scalar(128, 255, 255), blueImage);
	//cv::inRange(imageHSV, cv::Scalar(40, 100, 100), cv::Scalar(30, 255, 255), yellowImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 100), cv::Scalar(180, 18, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 150), cv::Scalar(360, 60, 255), whiteImage);
	//cv::medianBlur(whiteImage, whiteImage, 3);
	cv::medianBlur(redImage, redImage, 3);
	//cv::medianBlur(blueImage, blueImage, 3);
	//cv::medianBlur(yellowImage, yellowImage, 3);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 3 + 1, 2 * 3 + 1),
		cv::Point(3, 3));
	//erode(blueImage, blueImage, element);
	//dilate(blueImage, blueImage, element1);
//	fillHole(blueImage, blueImage);
//	erode(whiteImage, whiteImage, element);
//	dilate(whiteImage, whiteImage, element1);
//	fillHole(whiteImage, whiteImage);
	erode(redImage, redImage, element);
	dilate(redImage, redImage, element1);
//	fillHole(redImage, redImage);
//	erode(yellowImage, yellowImage, element);
	//dilate(yellowImage, yellowImage, element1);
//	fillHole(yellowImage, yellowImage);
	cv::Mat final = cv::Mat::zeros(originalImage.size(), CV_8UC3);
	cv::bitwise_and(originalImage, originalImage, final, redImage);
	return final;
}

cv::Mat imageSegmentation::highlightYellow(cv::Mat originalImage)
{   
	cv::Mat yellowImage;
	cv::Mat imageHSV;
	cv::cvtColor(originalImage, imageHSV, cv::COLOR_BGR2HSV);
	cv::inRange(imageHSV, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), yellowImage);
	cv::medianBlur(yellowImage, yellowImage, 3);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 3 + 1, 2 * 3 + 1),
		cv::Point(3, 3));
	erode(yellowImage, yellowImage, element);
	dilate(yellowImage, yellowImage, element1);
	cv::Mat newImage = cv::Mat::zeros(originalImage.size(), CV_8UC3);
	cv::bitwise_and(originalImage, originalImage, newImage, yellowImage);
	return newImage;
}

cv::Mat imageSegmentation::highlightBlue(cv::Mat originalImage)
{   
	cv::Mat blueImage;
	cv::Mat imageHSV;
	cv::cvtColor(originalImage, imageHSV, cv::COLOR_BGR2HSV);
	//cv::inRange(imageHSV, cv::Scalar(190, 130, 50), cv::Scalar(236, 255, 255), blueImage);
	//cv::inRange(imageHSV, cv::Scalar(120, 0, 0), cv::Scalar(170, 255, 255), blueImage);
	cv::inRange(imageHSV, cv::Scalar(90, 50, 70), cv::Scalar(128, 255, 255), blueImage);
	cv::medianBlur(blueImage, blueImage, 3);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 3 + 1, 2 * 3 + 1),
		cv::Point(3, 3));
	erode(blueImage, blueImage, element);
	dilate(blueImage, blueImage, element1);
	cv::Mat newImage = cv::Mat::zeros(originalImage.size(), CV_8UC3);
	cv::bitwise_and(originalImage, originalImage, newImage, blueImage);
	return newImage;
}

cv::Mat imageSegmentation::highlightWhite(cv::Mat originalImage)
{
	cv::Mat whiteImage;
	cv::Mat imageHSV;
	cv::cvtColor(originalImage, imageHSV, cv::COLOR_BGR2HSV);
	//cv::inRange(imageHSV, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 100), cv::Scalar(180, 18, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 240), cv::Scalar(255, 15, 255), whiteImage);
	//cv::inRange(imageHSV, cv::Scalar(0, 0, 150), cv::Scalar(360, 60, 255), whiteImage);
	cv::inRange(imageHSV, cv::Scalar(0, 0, 150), cv::Scalar(360, 60, 255), whiteImage);
	cv::medianBlur(whiteImage, whiteImage, 3);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 1 + 1, 2 * 1 + 1),
		cv::Point(1, 1));
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * 3 + 1, 2 * 3 + 1),
		cv::Point(3, 3));
	dilate(whiteImage, whiteImage, element1);
	cv::Mat newImage = cv::Mat::zeros(originalImage.size(), CV_8UC3);
	cv::bitwise_and(originalImage, originalImage, newImage, whiteImage);
	return newImage;
}