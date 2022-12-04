#include "mainwindow.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "MyTracker.h"
#include <QApplication>
using namespace cv;
using namespace std;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    if (argc != 2) {
         cout << "Incorrect args" << endl;
         return 0;
     }
    w.show();
    w.showTracking(argv[1]);
    return a.exec();
}
