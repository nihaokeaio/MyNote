#include "widget.h"
#include "ui_widget.h"
#include <QFile>
#include <QDebug>



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::FramelessWindowHint);//需要去掉标题栏
    this->setAttribute(Qt::WA_TranslucentBackground);

//    SingleLabel* secondLabelRight_=new SingleLabel(this);
//    secondLabelRight_->setSize(80,120);
//    secondLabelRight_->setRunSpeed(1);
//    secondLabelRight_->scroolStart();
//    secondLabelRight_->setPostion(QPoint(400,400));

//    SingleLabel* secondLabelLeft_=new SingleLabel(this);
//    secondLabelLeft_->setSize(80,120);
//    secondLabelLeft_->setRunSpeed(1);
//    secondLabelLeft_->scroolStart();

    QVector<SingleLabel*>TimerLabel;
    int speed=1;
    for(int i=0;i<4;i++)
    {
        SingleLabel* DisplayLabel_=new SingleLabel(this);
        DisplayLabel_->setSize(80,120);
        DisplayLabel_->setRunSpeed(speed);
        DisplayLabel_->setRunningCount(-1);
        DisplayLabel_->setInitNum(QString::number(0));
        if(i==0||i==2)
        {
            speed*=10;
            DisplayLabel_->setMaxNum(10);
        }
        else if(i==1||i==3)
        {
            speed*=6;
            DisplayLabel_->setMaxNum(6);
        }
        qDebug()<<speed;
        DisplayLabel_->setPostion(QPoint(400-(85*i),200));

        DisplayLabel_->scroolStart();
        TimerLabel.push_back(DisplayLabel_);
    }


}

Widget::~Widget()
{
    delete ui;
}

