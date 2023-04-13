#include "widget.h"
#include "ui_widget.h"
#include <QFile>
#include <QDebug>



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    secondLabel_=new SingleLabel(this);
    QLabel* helloLabel=new QLabel(this);
    helloLabel->setVisible(true);
    helloLabel->resize(100,100);
    helloLabel->setText("Hello world");
    helloLabel->move(50,50);
}

Widget::~Widget()
{
    delete ui;
}

