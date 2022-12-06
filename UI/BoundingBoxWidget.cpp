#include "BoundingBoxWidget.h"
BoundingBoxWidget::BoundingBoxWidget(QWidget *parent)
{
    QLabel *layout = new QLabel(this); // this keyword is iimportant, we tell the widget the parent withn be showed
    layout->setMargin(0);

    lblWatch = new QLabel(QString("00:00:00"));
    QString strFont = "font: 75 108pt 'Comic Sans MS',";
    lblWatch->setStyleSheet(strFont);
    lblWatch->setAlignment(Qt::AlignCenter);
    lblWatch->setMargin(0);




}