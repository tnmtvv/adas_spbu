#include "UI/mainwindow.h"
#include <iostream>
#include <QApplication>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    if (argc != 2) {
        cout << "Incorrect args" << endl;
        return 0;
    }
    w.show();
    if (w.setPathToVideo(argv[1])) {
        w.showFirstImage();
    } else {
        cerr << "Unable to open file!" << endl;
    }
    return a.exec();
}
