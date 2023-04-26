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
#include <QElapsedTimer>

class SingleLabel : public QWidget
{
public:
    explicit SingleLabel(QWidget *parent = nullptr);

    void init();
    void reloadAnimation(QParallelAnimationGroup *animalGroupFirst,const std::vector<QPoint>&posPoint);

    template< typename T>
    void swapWidget(T lWidget,T rWidget);

    void setRunSpeed(int scale=1.0);

    //\启动开关
    void scroolStart();

    void scroolStop();

    //调整位置
    void setPostion(const QPoint& pos);

    //调整大小
    void setSize(const QRect& rect);
    void setSize(int width,int height);

    void setopacity(QWidget *widget,qreal val);

    void setRunningCount(int count);

    void setInitNum(QString str);
    void setMaxNum(int number);
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
    int displayTime_=700;
    int runCount_=1;

    QString initNum_="0";
    int maxNum_=10;

    QPropertyAnimation* AnimalRightBlock_;
    QSequentialAnimationGroup* animalGroupRight_;
    int count_=0;
    int spaceGap_=5;

    bool moveDirection_=true;//1:up;0:down;
    qreal scale_ =1.0;


};

#endif // SINGLElABEL_H
