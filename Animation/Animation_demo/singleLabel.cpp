#include "singleLabel.h"
#include <QFile>
#include <QDebug>
#include <QMutex>
static QElapsedTimer timedebuge;//声明一个时钟对象
SingleLabel::SingleLabel(QWidget *parent):QWidget(parent)
{
    this->topLabel=new QLabel(this);
    this->midLabel=new QLabel(this);
    this->bottomLabel=new QLabel(this);




    MoveTime_=300;
    FadeTime_=200;
    BlockTime_=110;

    count_=0;
    spaceGap_=5;
    topLeftPos_={150,200};

    labelSize_=QRect(topLeftPos_.x(),topLeftPos_.y(),100,100);



}

void SingleLabel::init()
{
    this->topLabel->setGeometry(labelSize_);
    this->midLabel->setGeometry(labelSize_);
    this->bottomLabel->setGeometry(labelSize_);

    std::vector<QPoint>topLeftPostions={
        topLeftPos_,
        {topLeftPos_.x(),topLeftPos_.y()+labelSize_.height()+spaceGap_},
        {topLeftPos_.x(),topLeftPos_.y()+2*(labelSize_.height()+spaceGap_)}
    };

    this->topLabel->move(topLeftPostions[0]);
    this->midLabel->move(topLeftPostions[1]);
    this->bottomLabel->move(topLeftPostions[2]);

    this->topLabel->setText(QString::number((initNum_.toUInt()-2+maxNum_)%maxNum_));
    this->midLabel->setText(QString::number((initNum_.toUInt()-1+maxNum_)%maxNum_));
    this->bottomLabel->setText(QString::number((initNum_.toUInt()+maxNum_)%maxNum_));

    QFile file(":/myqss/class.qss");
    //只读方式打开文件
    file.open(QFile::ReadOnly);
    //读取文件的所有内容，并转换成QString类型
    QString styleSheet = tr(file.readAll());
    //当前窗口设置样式表
    this->setStyleSheet(styleSheet);

    QParallelAnimationGroup* animalGroupFirst=new QParallelAnimationGroup;
    this->reloadAnimation(animalGroupFirst,topLeftPostions);

    QSequentialAnimationGroup* animationGroup=new QSequentialAnimationGroup(this);


    //QPauseAnimation* AnimalPause=animationGroup->addPause(displayTime_);
//    AnimalRightGroupBlock->setDuration(displayTime_);


    animationGroup->addAnimation(animalGroupFirst);
    animationGroup->addPause(displayTime_);

    animationGroup->start();


    connect(animationGroup,&QSequentialAnimationGroup::finished,this,[=]{
        runCount_--;
//        qDebug()<<animationGroup->duration()<<animationGroup->currentAnimation();
//        qDebug()<<"animalGroupRight RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;
//        qDebug()<<"animalGroupRight RightMid : "<<this->midLabel->pos() << this->midLabel->text()<< this->midLabel;
//        qDebug()<<"animalGroupRight RightBottom : "<<this->bottomLabel->pos() << this->bottomLabel->text()<< this->bottomLabel;
        qDebug()<<animationGroup->currentTime();
        animationGroup->clear();

        QParallelAnimationGroup* animalGroupFirst=new QParallelAnimationGroup;
        this->reloadAnimation(animalGroupFirst,topLeftPostions);

//        QPropertyAnimation* AnimalRightGroupBlock=new QPropertyAnimation;
//        AnimalRightGroupBlock->setDuration(displayTime_);
//        animationGroup->addPause(displayTime_);
        animationGroup->addAnimation(animalGroupFirst);
        animationGroup->addPause(displayTime_);


        qDebug()<<"第一段程序耗时："<<timedebuge.elapsed()<<"ms "<<this->topLeftPos_;//输出计时


//        qDebug()<<"hello:"<<count;

        if(runCount_>0)
        {
            animationGroup->start();
            timedebuge.start();//开始计时
        }
        else
        {
            this->setopacity(this->topLabel,0.1);
            this->setopacity(this->midLabel,1);
            this->setopacity(this->bottomLabel,0.1);
        }
    });

}





void SingleLabel::reloadAnimation(QParallelAnimationGroup *AnimaGroup,const std::vector<QPoint>&posPoint)
{
    QPropertyAnimation* animalTopMove=moveUp(this->topLabel,labelSize_.height()/2+spaceGap_,MoveTime_/3);
    QPropertyAnimation* animalMidMove=moveUp(this->midLabel,labelSize_.height()+spaceGap_,MoveTime_);
    QPropertyAnimation* animalBottomLabelMove=moveUp(this->bottomLabel,labelSize_.height()+spaceGap_,MoveTime_);

    QPropertyAnimation* AnimalTopFade=fadeAway(this->topLabel,0.1,0.0,FadeTime_/2);
    QPropertyAnimation* AnimalMidFade=fadeAway(this->midLabel,1,0.1,FadeTime_);
    QPropertyAnimation* AnimalBottomFade=fadeAway(this->bottomLabel,0.1,1,FadeTime_);

    AnimaGroup->addAnimation(animalTopMove);
    AnimaGroup->addAnimation(animalMidMove);
    AnimaGroup->addAnimation(animalBottomLabelMove);

    AnimaGroup->addAnimation(AnimalTopFade);
    AnimaGroup->addAnimation(AnimalMidFade);
    AnimaGroup->addAnimation(AnimalBottomFade);

    AnimalRightBlock_=new QPropertyAnimation(this);
    AnimalRightBlock_->setDuration(BlockTime_);
    AnimalRightBlock_->setTargetObject(topLabel);
    AnimalRightBlock_->setStartValue(1);
    AnimalRightBlock_->setEndValue(1);

    AnimaGroup->addAnimation(AnimalRightBlock_);

    connect(AnimalRightBlock_,&QPropertyAnimation::finished,this,[=]{
//        qDebug()<<"address RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;
//        qDebug()<<"old RightUp : "<<this->topLabel->pos();
        this->topLabel->move(posPoint[2].x(),posPoint[2].y()+(labelSize_.height()/2));
        this->topLabel->setText(QString::number((this->bottomLabel->text().toInt()+1)%maxNum_));
//        qDebug()<<"New RightUp : "<<this->topLabel->pos();
        QPropertyAnimation* AnimalTopFadeSecond=fadeAway(this->topLabel,0.0,0.1,FadeTime_-BlockTime_);
        QPropertyAnimation* AnimalTopMoveSecond=moveUp(this->topLabel,labelSize_.height()/2,MoveTime_/2);
        AnimalTopFadeSecond->start(QAbstractAnimation::DeleteWhenStopped);
        AnimalTopMoveSecond->start(QAbstractAnimation::DeleteWhenStopped);

    });

    connect(AnimaGroup,&QParallelAnimationGroup::finished,this,[=]{
//        QLabel* label=this->this->topLabel;
//        this->this->topLabel=this->this->midLabel;
//        this->this->midLabel=this->this->bottomLabel;
//        qDebug()<<"address RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;

        swapWidget(&this->topLabel,&this->midLabel);
        swapWidget(&this->midLabel,&this->bottomLabel);

//        qDebug()<<"address RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;

//        qDebug()<<"RightUp : "<<this->topLabel->pos() << this->topLabel->text()<<this->topLabel;
//        qDebug()<<"RightMid : "<<this->midLabel->pos() << this->midLabel->text()<<this->midLabel;
//        qDebug()<<"RightBottom : "<<this->bottomLabel->pos() << this->bottomLabel->text()<<this->bottomLabel;

//        count++;
//        qDebug()<<"hello:"<<count;
    });


}

void SingleLabel::setRunSpeed(int scale)
{
//    this->scale_=scale;
//    this->MoveTime_*=scale;
//    this->FadeTime_*=scale;
//    this->BlockTime_*=scale;
    this->displayTime_=scale*1000-MoveTime_;
}

void SingleLabel::scroolStart()
{
    init();
}

void SingleLabel::scroolStop()
{
    setRunningCount(0);
}

void SingleLabel::setPostion(const QPoint &pos)
{
    this->topLeftPos_=pos;
}

void SingleLabel::setSize(const QRect &rect)
{
    this->labelSize_=rect;
}

void SingleLabel::setSize(int width, int height)
{
    this->labelSize_=QRect(this->topLeftPos_,QSize(width,height));
}

void SingleLabel::setopacity(QWidget *widget, qreal val)
{
    QGraphicsOpacityEffect* my_widgetProcessOpacity=new QGraphicsOpacityEffect;
    my_widgetProcessOpacity->setParent(widget);
    my_widgetProcessOpacity->setOpacity(val);
    widget->setGraphicsEffect(my_widgetProcessOpacity);

}

void SingleLabel::setRunningCount(int count)
{
    if(count==-1)
    {
        this->runCount_=INT_MAX;
    }
    else
    {
        this->runCount_=count;
    }

}

void SingleLabel::setInitNum(QString str)
{
    initNum_=str;
}

void SingleLabel::setMaxNum(int number)
{
    maxNum_=number;
}


template< typename T>
void SingleLabel::swapWidget(T lWidget,T rWidget)
{
      QString tempObjName=(*lWidget)->objectName();

      auto  temp=*lWidget;
      *lWidget=*rWidget;
      *rWidget=temp;
      temp=nullptr;

      (*rWidget)->setObjectName((*lWidget)->objectName());
      (*lWidget)->setObjectName(tempObjName);

//      qDebug()<<"after address lWidget : "<<*lWidget<<"address rWidget : "<<*rWidget;
}



QPropertyAnimation* SingleLabel::moveUp(QWidget *widget,qreal span,qreal moveTime)
{
    QPropertyAnimation* pAnimationAlpha=new QPropertyAnimation();
    pAnimationAlpha->setTargetObject(widget);
    pAnimationAlpha->setPropertyName("geometry");
    pAnimationAlpha->setDuration(moveTime);

    pAnimationAlpha->setStartValue(QRect(widget->pos().x(), widget->pos().y(), widget->width(), widget->height()));
    pAnimationAlpha->setEndValue(QRect(widget->pos().x() , widget->pos().y()-span, widget->width(), widget->height()));

    if(moveDirection_)
    {
        pAnimationAlpha->setDirection(QAbstractAnimation::Forward);
    }
    else
    {
        pAnimationAlpha->setDirection(QAbstractAnimation::Backward);
    }

    return  pAnimationAlpha;
}

QPropertyAnimation* SingleLabel::fadeAway(QWidget *widget,qreal startValue,qreal endValue,qreal fadeTime)
{
    QGraphicsOpacityEffect* my_widgetProcessOpacity=new QGraphicsOpacityEffect;
    my_widgetProcessOpacity->setParent(widget);
    widget->setGraphicsEffect(my_widgetProcessOpacity);

    QPropertyAnimation* pAnimationAlphashow=new QPropertyAnimation;
    pAnimationAlphashow->setParent(widget);
    pAnimationAlphashow->setTargetObject(my_widgetProcessOpacity);
    pAnimationAlphashow->setPropertyName("opacity");
    pAnimationAlphashow->setDuration(fadeTime);
    pAnimationAlphashow->setStartValue(startValue);
    pAnimationAlphashow->setEndValue(endValue);
    pAnimationAlphashow->setEasingCurve(QEasingCurve::Linear);

    return pAnimationAlphashow;
}
