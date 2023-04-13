#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <QAnimationGroup>
#include <vector>

#include "singleLabel.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();
//    QPropertyAnimation* moveUp(QWidget* widget,qreal span,int costTime);
//    QPropertyAnimation* fadeAway(QWidget* widget,bool Direction,int costTime,qreal startValue,qreal endValue);
//    QPropertyAnimation* moveDown(QWidget *widget,qreal span,int costTime);
//    template< typename T>
//    void swapWidget(T lWidget,T rWidget);

//    void reloadAnimation(QParallelAnimationGroup* animalGroupFirst,const vector<QPoint>&posPoint );
private:
    Ui::Widget *ui;
//    int movex=350,movey=300,spaceGap=5;

//    int costMoveTime=300;
//    int costFadeTime=200;
//    int costBlockTime=costMoveTime/3+10;

//    QPropertyAnimation* AnimalRightBlock;
//    QSequentialAnimationGroup* animalGroupRight;
//    int count=0;
    SingleLabel* secondLabel_;

};

#endif // WIDGET_H
