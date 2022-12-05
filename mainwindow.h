#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "MyTracker.h"
#include "WindowState.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

    void showTracking(const std::string &path);

private:
    WindowState state;
    Ui::MainWindow *ui;

    void keyPressEvent(QKeyEvent *e) override;

};

#endif // MAINWINDOW_H
