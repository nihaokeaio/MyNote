#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <QAnimationGroup>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();
    QPropertyAnimation* moveUp(QWidget* widget,qreal span,int costTime);
    QPropertyAnimation* fadeAway(QWidget* widget,bool Direction,int costTime,qreal startValue,qreal endValue);
    QPropertyAnimation* moveDown(QWidget *widget,qreal span,int costTime);
    template< typename T>
    void swapWidget(T lWidget,T rWidget);
private:
    Ui::Widget *ui;
    int movex=350,movey=300,spaceGap=5;

};

#endif // WIDGET_H
