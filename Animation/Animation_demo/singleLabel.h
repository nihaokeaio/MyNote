#ifndef SINGLElABEL_H
#define SINGLElABEL_H

#include <QWidget>
#include <QLabel>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <QAnimationGroup>
#include <vector>

class SingleLabel : public QWidget
{
public:
    explicit SingleLabel(QWidget *parent = nullptr);

    void init();
    void reloadAnimation(QParallelAnimationGroup *animalGroupFirst,const std::vector<QPoint>&posPoint);

    template< typename T>
    void swapWidget(T lWidget,T rWidget);
private:
    QPropertyAnimation* moveUp(QWidget *widget,qreal span,qreal moveTime=300);
    QPropertyAnimation* fadeAway(QWidget *widget,qreal startValue,qreal endValue,qreal fadeTime=200);
private:
    QLabel* topLabel;
    QLabel* midLabel;
    QLabel* bottomLabel;

    QPoint topLeftPos_;
    QRect labelSize_;

    int MoveTime_=300;
    int FadeTime_=200;
    int BlockTime_=110;

    QPropertyAnimation* AnimalRightBlock_;
    QSequentialAnimationGroup* animalGroupRight_;
    int count_=0;
    int spaceGap_=5;

    bool moveDirection=true;//1:up;0:down;
};

#endif // SINGLElABEL_H
