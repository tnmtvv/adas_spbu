#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "QKeyEvent"
#include "WindowState.h"
#include "MyTracker.h"
#include <QPushButton>


using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow) {
    setWindowTitle("Pedestrian tracking labeler");
    state = WindowState::WAITING_FOR_SELECTION;
    ui->setupUi(this);
    ui->horizontalSlider->setEnabled(false);
}

MainWindow::~MainWindow() {
    delete ui;
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
            std::cout << "!!!" << std::endl;
            break;
    }
}

void MainWindow::showTracking(const std::string &path) {

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
    int nFrame = 1;
    for (int i = 0; i < nFrame; i++) { capture >> frame; }
    box = selectROI("Frame", frame);
    destroyWindow("Frame");
    MyTracker tracker = MyTracker();
    tracker.startTracking(path, box, nFrame);
    while (!frame.empty()) {
        if (state != WindowState::ON_PAUSE) {
            box = tracker.getNextPedestrianPosition();
            capture >> frame;
            rectangle(frame, box, cv::Scalar(0, 255, 0), 5);
            cvtColor(frame, frame, COLOR_BGR2RGB);
            image = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            image = image.scaledToHeight(ui->label->height());
            image = image.scaledToWidth(ui->label->width());
            ui->label->setPixmap(QPixmap::fromImage(image));
        }
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
}

