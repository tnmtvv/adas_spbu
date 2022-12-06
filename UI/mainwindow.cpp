#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "QKeyEvent"
#include "WindowState.h"
#include "trackers/MyTracker.h"
#include <QPushButton>
#include <QPainter>

using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow) {
    setWindowTitle("Pedestrian tracking labeler");
    state = WindowState::SLIDING;
    ui->setupUi(this);
    ui->horizontalSlider->setEnabled(true);
    connect(ui->horizontalSlider, &QSlider::sliderMoved, this, &MainWindow::onSliderValueChanged);
    connect(ui->selectROI, &QPushButton::clicked, this, &MainWindow::onSelectROIClick);
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onButtonClick);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::onSelectROIClick() {
    state = WindowState::TRACKING;
    showTracking(ui->horizontalSlider->value() + 1);
}

void MainWindow::onSliderValueChanged() {
    QImage img = getImageByIndex(ui->horizontalSlider->value());
    setImageToLabel(img);
}

QImage MainWindow::getImageByIndex(int index) {
    capture.set(CAP_PROP_POS_FRAMES, index);
    Mat frame;
    capture >> frame;
    return convertOpenCVImageToQt(frame);
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    switch (event->key()) {
        case Qt::Key_S:
            if (state != WindowState::ON_PAUSE) {
                state = WindowState::ON_PAUSE;
                ui->horizontalSlider->setEnabled(true);
            } else {
                state = WindowState::TRACKING;
                ui->horizontalSlider->setEnabled(false);
            }
            break;
    }
}

void MainWindow::showTracking(int nFrame) {

    const string windowName = "Choose the tracking rectangle";
    namedWindow(windowName, WINDOW_AUTOSIZE);
    cv::Rect2d box;
    Mat frame;
    capture.set(CAP_PROP_POS_FRAMES, nFrame);
    capture >> frame;
    box = selectROI(windowName, frame);
    destroyWindow(windowName);
    MyTracker tracker = MyTracker();
    tracker.startTracking(path, box, nFrame);
    while (!frame.empty()) {
        if (state == WindowState::TRACKING) {
            box = tracker.getNextPedestrianPosition();
            capture >> frame;
            nFrame++;
            rectangle(frame, box, cv::Scalar(0, 255, 0), 5);
            setImageToLabel(convertOpenCVImageToQt(frame));
            ui->horizontalSlider->setValue(nFrame);
        }
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
}

QImage MainWindow::convertOpenCVImageToQt(const cv::Mat &frame) {
    QImage image;
    cvtColor(frame, frame, COLOR_BGR2RGB);
    image = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
    image = image.scaledToHeight(ui->label->height());
    image = image.scaledToWidth(ui->label->width());
    return image;
}

bool MainWindow::setPathToVideo(const string &pathToVideo) {
    capture = VideoCapture(pathToVideo);
    if (!capture.isOpened()) {
        return false;
    }
    path = pathToVideo;
    nFrames = capture.get(CAP_PROP_FRAME_COUNT);
    ui->horizontalSlider->setMaximum(nFrames - 1);
    std::cout << "frame total: " << nFrames << std::endl;
    return true;
}

void MainWindow::setImageToLabel(const QImage &img) {
    ui->label->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::showFirstImage() {
    setImageToLabel(getImageByIndex(1));
}

void MainWindow::onButtonClick() {
    repaint();
}

void MainWindow::paintEvent(QPaintEvent *event) {

}

