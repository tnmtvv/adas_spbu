#ifndef TRACKING_BOUNDINGBOXWIDGET_H
#define TRACKING_BOUNDINGBOXWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QLabel>

QT_BEGIN_NAMESPACE
namespace Ui { class BoundingBoxWidget; }
QT_END_NAMESPACE

class BoundingBoxWidget : public QWidget {
public:
    BoundingBoxWidget(QWidget *parent = nullptr);

private :
    QLabel *lblWatch;
};

#endif //TRACKING_BOUNDINGBOXWIDGET_H
