#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "MyTracker.h"
using namespace cv;
using namespace std;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showTracking(std::string path){
    VideoCapture capture(path);
     if (!capture.isOpened()) {
         cerr << "Unable to open file!" << endl;
         return;
     }
     namedWindow("Frame", WINDOW_AUTOSIZE);
     cv::Rect2d box;
     QImage image;
     Mat frame;
     QPixmap pix;
     int nFrame = 150;
     for (int i = 0; i < nFrame; i++){ capture >> frame;}
     box = selectROI("Frame", frame);
     destroyWindow("Frame");
     MyTracker tracker = MyTracker();
     tracker.startTracking(path, box, nFrame);
     while (!frame.empty()) {
         box = tracker.getNextPedestrianPosition();
         capture >> frame;
         rectangle(frame, box, cv::Scalar(0, 255, 0));
         cvtColor(frame, frame, COLOR_BGR2RGB);
         image = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
         image = image.scaledToWidth(1920);
         image = image.scaledToHeight(1080);

         ui->label->setPixmap(QPixmap::fromImage(image));
         int keyboard = waitKey(30);
         if (keyboard == 'q' || keyboard == 27)
             break;
     }
}

