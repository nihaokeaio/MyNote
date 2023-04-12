#include "widget.h"
#include "ui_widget.h"
#include <QFile>
#include <QDebug>



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    ui->label_LeftUp->resize(100,100);
    ui->label_LeftMid->resize(100,100);
    ui->label_RightUp->resize(100,100);
    ui->label_RightMid->resize(100,100);
    ui->label_LeftButtom->resize(100,100);
    ui->label_RightButtom->resize(100,100);



    vector<QPoint>posPoint={
        {movex,movey},
        {movex,movey+ui->label_LeftUp->height()+spaceGap},
        {movex,movey+ui->label_LeftUp->height()+ui->label_LeftMid->height()+2*spaceGap},

        {movex+ui->label_LeftUp->width()+spaceGap,movey},
        {movex+ui->label_LeftUp->width()+spaceGap,movey+ui->label_RightMid->height()+spaceGap},

        {movex+ui->label_LeftUp->width()+spaceGap,movey+ui->label_RightUp->height()+ui->label_RightMid->height()+2*spaceGap}
    };




    ui->label_LeftUp->move(posPoint[0]);
    ui->label_LeftMid->move(posPoint[1]);
    ui->label_LeftButtom->move(posPoint[2]);

    ui->label_RightUp->move(posPoint[3]);
    ui->label_RightMid->move(posPoint[4]);
    ui->label_RightButtom->move(posPoint[5]);


    ui->label_LeftUp->setText("0");
    ui->label_LeftMid->setText("1");
    ui->label_LeftButtom->setText("");
    ui->label_RightUp->setText("0");
    ui->label_RightMid->setText("1");
    ui->label_RightButtom->setText("2");

    ui->labelUnderline->setText("");

    ui->label_LeftUp->setAlignment(Qt::AlignHCenter);
    ui->label_LeftMid->setAlignment(Qt::AlignHCenter);
    ui->label_RightUp->setAlignment(Qt::AlignHCenter);
    ui->label_RightMid->setAlignment(Qt::AlignHCenter);
    ui->label_LeftButtom->setAlignment(Qt::AlignHCenter);
    ui->label_RightButtom->setAlignment(Qt::AlignHCenter);

    //ui->label_RightButtom->setHidden(true);



    qDebug()<<ui->label_LeftUp->pos();
    qDebug()<<ui->label_LeftMid->mapToGlobal(ui->label_LeftMid->pos());
    ui->labelUnderline->move(movex,ui->label_LeftButtom->geometry().bottomLeft().y()+spaceGap);

    QFile file(":/myqss/class.qss");
    //只读方式打开文件
    file.open(QFile::ReadOnly);
    //读取文件的所有内容，并转换成QString类型
    QString styleSheet = tr(file.readAll());
    //当前窗口设置样式表
    this->setStyleSheet(styleSheet);
    qDebug()<<"this objectname:"<<this->objectName();



    QParallelAnimationGroup* animalGroupFirst=new QParallelAnimationGroup;


    this->reloadAnimation(animalGroupFirst,posPoint);

    QSequentialAnimationGroup* animalGroupRight=new QSequentialAnimationGroup(this);


    QPropertyAnimation* AnimalRightGroupBlock=new QPropertyAnimation;
    AnimalRightGroupBlock->setDuration(700);


    animalGroupRight->addAnimation(animalGroupFirst);
    animalGroupRight->addAnimation(AnimalRightGroupBlock);

    animalGroupRight->start();



    connect(animalGroupRight,&QSequentialAnimationGroup::finished,this,[=]{
        qDebug()<<animalGroupRight->duration()<<animalGroupRight->currentAnimation();
        qDebug()<<"animalGroupRight RightUp : "<<ui->label_RightUp->pos() << ui->label_RightUp->text() << ui->label_RightUp;
        qDebug()<<"animalGroupRight RightMid : "<<ui->label_RightMid->pos() << ui->label_RightMid->text()<< ui->label_RightMid;
        qDebug()<<"animalGroupRight RightButtom : "<<ui->label_RightButtom->pos() << ui->label_RightButtom->text()<< ui->label_RightButtom;
        animalGroupRight->clear();

        QParallelAnimationGroup* animalGroupFirst=new QParallelAnimationGroup;
        this->reloadAnimation(animalGroupFirst,posPoint);

        QPropertyAnimation* AnimalRightGroupBlock=new QPropertyAnimation;
        AnimalRightGroupBlock->setDuration(700);
        animalGroupRight->addAnimation(animalGroupFirst);
        animalGroupRight->addAnimation(AnimalRightGroupBlock);

        qDebug()<<"hello:"<<count;

        animalGroupRight->start();


    });

    //和动画结束的时间有关，不然对象就会被锁导致无法共享使用







}

Widget::~Widget()
{
    delete ui;
}

QPropertyAnimation* Widget::moveUp(QWidget *widget,qreal span,int costTime)
{
    qDebug()<<"move widget->pos() : "<<widget->pos() <<"widget->name()"<<widget->objectName();
    QPropertyAnimation* pAnimationAlpha=new QPropertyAnimation();
    pAnimationAlpha->setTargetObject(widget);
    pAnimationAlpha->setPropertyName("geometry");
    pAnimationAlpha->setDuration(costTime);

    pAnimationAlpha->setStartValue(QRect(widget->pos().x(), widget->pos().y(), widget->width(), widget->height()));

    pAnimationAlpha->setEndValue(QRect(widget->pos().x() , widget->pos().y()-span, widget->width(), widget->height()));
    //pAnimationAlpha->setLoopCount(-1);  //永远运行，直到stop
//    pAnimationAlpha->start(QAbstractAnimation::DeleteWhenStopped);
    return  pAnimationAlpha;
}

QPropertyAnimation* Widget::fadeAway(QWidget *widget,bool Direction,int costTime,qreal startValue,qreal endValue)
{
    qDebug()<<"widget->pos() : "<<widget->pos() <<"widget->name()"<<widget->objectName();
    QGraphicsOpacityEffect* my_widgetProcessOpacity=new QGraphicsOpacityEffect;
    my_widgetProcessOpacity->setParent(widget);
    my_widgetProcessOpacity->setOpacity(0);
    widget->setGraphicsEffect(my_widgetProcessOpacity);

    QPropertyAnimation* pAnimationAlphashow=new QPropertyAnimation();
    pAnimationAlphashow->setParent(widget);
    pAnimationAlphashow->setTargetObject(my_widgetProcessOpacity);
    pAnimationAlphashow->setPropertyName("opacity");
    pAnimationAlphashow->setDuration(costTime);
    pAnimationAlphashow->setStartValue(startValue);
    pAnimationAlphashow->setEndValue(endValue);
    if(Direction)
    {
        pAnimationAlphashow->setDirection(QAbstractAnimation::Forward);
    }
    else
    {
        pAnimationAlphashow->setDirection(QAbstractAnimation::Backward);
    }

    pAnimationAlphashow->setEasingCurve(QEasingCurve::Linear);
//    pAnimationAlphashow->start(QAbstractAnimation::DeleteWhenStopped);
    return pAnimationAlphashow;
}


QPropertyAnimation* Widget::moveDown(QWidget *widget,qreal span,int costTime)
{


    QPropertyAnimation* pAnimationAlphashow=new QPropertyAnimation();
    pAnimationAlphashow->setTargetObject(widget);
    pAnimationAlphashow->setPropertyName("geometry");
    pAnimationAlphashow->setDuration(costTime);

    pAnimationAlphashow->setStartValue(QRect(widget->pos().x(), widget->pos().y(), widget->width(), widget->height()));

    pAnimationAlphashow->setEndValue(QRect(widget->pos().x() , widget->pos().y()-span-spaceGap, widget->width(), widget->height()));
    pAnimationAlphashow->setEasingCurve(QEasingCurve::Linear);

    return pAnimationAlphashow;
}

void Widget::reloadAnimation(QParallelAnimationGroup *animalGroupFirst,const vector<QPoint>&posPoint)
{
    QPropertyAnimation* AnimalRightUpMove=moveUp(ui->label_RightUp,ui->label_RightUp->width()+spaceGap,costMoveTime/3);
    QPropertyAnimation* AnimalRightMidMove=moveUp(ui->label_RightMid,ui->label_RightMid->width()+spaceGap,costMoveTime);
    QPropertyAnimation* AnimalRightButtomMove=moveUp(ui->label_RightButtom,ui->label_RightButtom->width()+spaceGap,costMoveTime);

    QPropertyAnimation* AnimalRightUpFade=fadeAway(ui->label_RightUp,true,costFadeTime/2,0.1,0.0);
    QPropertyAnimation* AnimalRightMidFade=fadeAway(ui->label_RightMid,true,costFadeTime,1,0.1);
    QPropertyAnimation* AnimalRightButtomFade=fadeAway(ui->label_RightButtom,true,costFadeTime,0.1,1);

    animalGroupFirst->addAnimation(AnimalRightUpMove);
    animalGroupFirst->addAnimation(AnimalRightMidMove);
    animalGroupFirst->addAnimation(AnimalRightButtomMove);

    animalGroupFirst->addAnimation(AnimalRightUpFade);
    animalGroupFirst->addAnimation(AnimalRightMidFade);
    animalGroupFirst->addAnimation(AnimalRightButtomFade);

    AnimalRightBlock=new QPropertyAnimation;
    AnimalRightBlock->setDuration(costBlockTime);

    connect(AnimalRightBlock,&QPropertyAnimation::finished,this,[=]{
        qDebug()<<"address RightUp : "<<ui->label_RightUp->pos() << ui->label_RightUp->text() << ui->label_RightUp;
        qDebug()<<"old RightUp : "<<ui->label_RightUp->pos();
        this->ui->label_RightUp->move(posPoint[5].x(),posPoint[5].y()+(ui->label_RightUp->height()/2));
        //this->ui->label_RightUp->setGeometry(posPoint[5].x(),posPoint[5].y()+(ui->label_RightUp->height()/2),100,100);
        this->ui->label_RightUp->setText(QString::number(ui->label_RightButtom->text().toInt()+1));
        qDebug()<<"New RightUp : "<<ui->label_RightUp->pos();
        QPropertyAnimation* AnimalRightUpFadeSecond=fadeAway(ui->label_RightUp,true,costFadeTime-costBlockTime,0.0,0.1);
        QPropertyAnimation* AnimalRightUpMoveSecond=moveUp(ui->label_RightUp,ui->label_RightUp->height()/2,costMoveTime/2);
        AnimalRightUpFadeSecond->start(QAbstractAnimation::DeleteWhenStopped);
        AnimalRightUpMoveSecond->start(QAbstractAnimation::DeleteWhenStopped);

    });

    connect(animalGroupFirst,&QParallelAnimationGroup::finished,this,[=]{
//        QLabel* label=this->ui->label_RightUp;
//        this->ui->label_RightUp=this->ui->label_RightMid;
//        this->ui->label_RightMid=this->ui->label_RightButtom;
        qDebug()<<"address RightUp : "<<ui->label_RightUp->pos() << ui->label_RightUp->text() << ui->label_RightUp;

        swapWidget(&this->ui->label_RightUp,&this->ui->label_RightMid);
        swapWidget(&this->ui->label_RightButtom,&this->ui->label_RightMid);

        qDebug()<<"address RightUp : "<<ui->label_RightUp->pos() << ui->label_RightUp->text() << ui->label_RightUp;

        qDebug()<<"RightUp : "<<ui->label_RightUp->pos() << ui->label_RightUp->text()<<ui->label_RightUp;
        qDebug()<<"RightMid : "<<ui->label_RightMid->pos() << ui->label_RightMid->text()<<ui->label_RightMid;
        qDebug()<<"RightButtom : "<<ui->label_RightButtom->pos() << ui->label_RightButtom->text()<<ui->label_RightButtom;

        count++;
        qDebug()<<"hello:"<<count;
    });

    animalGroupFirst->addAnimation(AnimalRightBlock);
}

template< typename T>
void Widget::swapWidget(T lWidget,T rWidget)
{
//    QRect lrect=lWidget->geometry();
//    lWidget->setGeometry(rWidget->geometry());
//    rWidget->setGeometry(lrect);
//    swap(lWidget,rWidget);
      QString tempObjName=(*lWidget)->objectName();
      qDebug()<<"before address lWidget : "<<*lWidget<<"address rWidget : "<<*rWidget;

      auto  temp=*lWidget;
      *lWidget=*rWidget;
      *rWidget=temp;
      temp=nullptr;
//      QString temp=static_cast<QLabel*>(lWidget)->text();
//      static_cast<QLabel*>(rWidget)->setText(static_cast<QLabel*>(rWidget)->text());
//      static_cast<QLabel*>(rWidget)->setText(temp);

      (*rWidget)->setObjectName((*lWidget)->objectName());
      (*lWidget)->setObjectName(tempObjName);

      qDebug()<<"after address lWidget : "<<*lWidget<<"address rWidget : "<<*rWidget;
}
