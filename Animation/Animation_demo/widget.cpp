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

    int movex=150,movey=70,spaceGap=5;
    ui->label_LeftUp->move(movex,movey);
    ui->label_LeftMid->move(movex,movey+ui->label_LeftUp->height()+spaceGap);
    ui->label_RightUp->move(movex+ui->label_LeftUp->width()+spaceGap,movey);
    ui->label_RightMid->move(movex+ui->label_LeftUp->width()+spaceGap,movey+ui->label_LeftUp->height()+spaceGap);
    ui->label_LeftButtom->move(movex,ui->label_RightMid->pos().y()+ui->label_RightMid->height()+spaceGap);
    ui->label_RightButtom->move(movex+ui->label_LeftUp->width()+spaceGap,ui->label_RightMid->pos().y()+ui->label_RightMid->height()+spaceGap);


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

    QPropertyAnimation* AnimalRightUpMove=moveUp(ui->label_RightUp);
    QPropertyAnimation* AnimalRightMidMove=moveUp(ui->label_RightMid);
    QPropertyAnimation* AnimalRightButtomMove=moveUp(ui->label_RightButtom);;

    int costTime=200;
    QPropertyAnimation* AnimalRightUpFade=fadeAway(ui->label_RightUp,true,costTime,1,0.1);
    QPropertyAnimation* AnimalRightButtomFade=fadeAway(ui->label_RightButtom,true,costTime,0,0.1);
    //fadeAway(ui->label_RightMid);

    QSequentialAnimationGroup* animalGroupFirst=new QSequentialAnimationGroup;






}

Widget::~Widget()
{
    delete ui;
}

QPropertyAnimation* Widget::moveUp(QWidget *widget)
{
    QPropertyAnimation* pAnimationAlpha=new QPropertyAnimation();
    pAnimationAlpha->setTargetObject(widget);
    pAnimationAlpha->setPropertyName("geometry");
    pAnimationAlpha->setDuration(300);

    pAnimationAlpha->setStartValue(QRect(widget->pos().x(), widget->pos().y(), widget->width(), widget->height()));

    pAnimationAlpha->setEndValue(QRect(widget->pos().x() , widget->pos().y()-widget->width()/2, widget->width(), widget->height()));
    //pAnimationAlpha->setLoopCount(-1);  //永远运行，直到stop
    pAnimationAlpha->start(QAbstractAnimation::DeleteWhenStopped);
}

QPropertyAnimation* Widget::fadeAway(QWidget *widget,bool Direction,int costTime,qreal startValue,qreal endValue)
{
    QGraphicsOpacityEffect* my_widgetProcessOpacity;
    my_widgetProcessOpacity=new QGraphicsOpacityEffect(widget);
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
    pAnimationAlphashow->start(QAbstractAnimation::DeleteWhenStopped);
}
