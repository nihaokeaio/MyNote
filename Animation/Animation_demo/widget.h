#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
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
    QPropertyAnimation* moveUp(QWidget* widget);
    QPropertyAnimation* fadeAway(QWidget* widget,bool Direction,int costTime,qreal startValue,qreal endValue);
private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
