#include "singleLabel.h"
#include <QFile>
#include <QDebug>

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

    init();
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

    this->topLabel->setText("9");
    this->midLabel->setText("0");
    this->bottomLabel->setText("1");

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


    QPropertyAnimation* AnimalRightGroupBlock=new QPropertyAnimation;
    AnimalRightGroupBlock->setDuration(700);


    animationGroup->addAnimation(animalGroupFirst);
    animationGroup->addAnimation(AnimalRightGroupBlock);

    animationGroup->start();


    connect(animationGroup,&QSequentialAnimationGroup::finished,this,[=]{
        qDebug()<<animationGroup->duration()<<animationGroup->currentAnimation();
        qDebug()<<"animalGroupRight RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;
        qDebug()<<"animalGroupRight RightMid : "<<this->midLabel->pos() << this->midLabel->text()<< this->midLabel;
        qDebug()<<"animalGroupRight RightBottom : "<<this->bottomLabel->pos() << this->bottomLabel->text()<< this->bottomLabel;
        animationGroup->clear();

        QParallelAnimationGroup* animalGroupFirst=new QParallelAnimationGroup;
        this->reloadAnimation(animalGroupFirst,topLeftPostions);

        QPropertyAnimation* AnimalRightGroupBlock=new QPropertyAnimation;
        AnimalRightGroupBlock->setDuration(700);
        animationGroup->addAnimation(animalGroupFirst);
        animationGroup->addAnimation(AnimalRightGroupBlock);

//        qDebug()<<"hello:"<<count;

        animationGroup->start();


    });

}





void SingleLabel::reloadAnimation(QParallelAnimationGroup *AnimaGroup,const std::vector<QPoint>&posPoint)
{
    QPropertyAnimation* animalTopMove=moveUp(this->topLabel,labelSize_.height()/2+spaceGap_,MoveTime_/3);
    QPropertyAnimation* animalMidMove=moveUp(this->midLabel,labelSize_.height()+spaceGap_);
    QPropertyAnimation* animalBottomLabelMove=moveUp(this->bottomLabel,labelSize_.height()+spaceGap_);

    QPropertyAnimation* AnimalTopFade=fadeAway(this->topLabel,0.1,0.0,FadeTime_/2);
    QPropertyAnimation* AnimalMidFade=fadeAway(this->midLabel,1,0.1);
    QPropertyAnimation* AnimalBottomFade=fadeAway(this->bottomLabel,0.1,1);

    AnimaGroup->addAnimation(animalTopMove);
    AnimaGroup->addAnimation(animalMidMove);
    AnimaGroup->addAnimation(animalBottomLabelMove);

    AnimaGroup->addAnimation(AnimalTopFade);
    AnimaGroup->addAnimation(AnimalMidFade);
    AnimaGroup->addAnimation(AnimalBottomFade);

    AnimalRightBlock_=new QPropertyAnimation;
    AnimalRightBlock_->setDuration(BlockTime_);

    connect(AnimalRightBlock_,&QPropertyAnimation::finished,this,[=]{
        qDebug()<<"address RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;
        qDebug()<<"old RightUp : "<<this->topLabel->pos();
        this->topLabel->move(posPoint[2].x(),posPoint[2].y()+(labelSize_.height()/2+spaceGap_));
        this->topLabel->setText(QString::number((this->bottomLabel->text().toInt()+1)%10));
        qDebug()<<"New RightUp : "<<this->topLabel->pos();
        QPropertyAnimation* AnimalTopFadeSecond=fadeAway(this->topLabel,0.0,0.1,FadeTime_-BlockTime_);
        QPropertyAnimation* AnimalTopMoveSecond=moveUp(this->topLabel,labelSize_.height()/2,MoveTime_/2);
        AnimalTopFadeSecond->start(QAbstractAnimation::DeleteWhenStopped);
        AnimalTopMoveSecond->start(QAbstractAnimation::DeleteWhenStopped);

    });

    connect(AnimaGroup,&QParallelAnimationGroup::finished,this,[=]{
//        QLabel* label=this->this->topLabel;
//        this->this->topLabel=this->this->midLabel;
//        this->this->midLabel=this->this->bottomLabel;
        qDebug()<<"address RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;

        swapWidget(&this->topLabel,&this->midLabel);
        swapWidget(&this->midLabel,&this->bottomLabel);

        qDebug()<<"address RightUp : "<<this->topLabel->pos() << this->topLabel->text() << this->topLabel;

        qDebug()<<"RightUp : "<<this->topLabel->pos() << this->topLabel->text()<<this->topLabel;
        qDebug()<<"RightMid : "<<this->midLabel->pos() << this->midLabel->text()<<this->midLabel;
        qDebug()<<"RightBottom : "<<this->bottomLabel->pos() << this->bottomLabel->text()<<this->bottomLabel;

//        count++;
//        qDebug()<<"hello:"<<count;
    });

    AnimaGroup->addAnimation(AnimalRightBlock_);
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

    if(moveDirection)
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
