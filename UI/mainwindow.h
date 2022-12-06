#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "trackers/MyTracker.h"
#include "WindowState.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

    void showTracking(int nFrame);

    void showFirstImage();

    bool setPathToVideo(const std::string &pathToVideo);

private:
    std::string path;
    int nFrames;
    WindowState state;
    Ui::MainWindow *ui;
    cv::VideoCapture capture;

    QImage getImageByIndex(int index);

    void setImageToLabel(const QImage &img);

    void onSliderValueChanged();

    void onSelectROIClick();

    void keyPressEvent(QKeyEvent *e) override;

    void onButtonClick();

    void paintEvent(QPaintEvent *event) override;

    QImage convertOpenCVImageToQt(const cv::Mat &frame);
};

#endif // MAINWINDOW_H
