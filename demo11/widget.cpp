#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    startcharts=new QPushButton;
    startAnimation=new QPushButton(this);

    startAnimation->setText("开始动画");
    startcharts->setText("绘画图表");
    QHBoxLayout* hlay=new QHBoxLayout;
    hlay->addWidget(startAnimation);
    hlay->addSpacing(10);
    hlay->addWidget(startcharts);

    //this->setWindowFlags(Qt::FramelessWindowHint);
    ui->displayWidget->setBackgroundBrush(QBrush(Qt::red));
    connect(startcharts,&QPushButton::clicked,this,&Widget::slotStartcharts);
    connect(startAnimation,&QPushButton::clicked,this,&Widget::slotStartAnimation);


    QVBoxLayout* vlay=new QVBoxLayout(this);
    vlay->addLayout(hlay);
    vlay->addWidget(ui->displayWidget);
    vlay->addWidget(ui->chartsWidget);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::animationChange(QVariant value)
{
    ++time;
    datalist.append(QPointF(time,value.toInt()));
}

void Widget::slotStartcharts()
{
    QSplineSeries* series=new QSplineSeries;
    series->append(datalist);

    chart=new QChart;
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->axes(Qt::Vertical).first()->setRange(0,400);
    chart->axes(Qt::Horizontal).first()->setRange(0,time);
    ui->chartsWidget->setChart(chart);
}

void Widget::slotStartAnimation()
{
    static int resizeValue{500};
    resizeValue = -resizeValue;
    animation = new QPropertyAnimation(ui->displayWidget, "geometry");
    animation->setDuration(500);
    animation->setStartValue(QRect(ui->displayWidget->pos().x(), ui->displayWidget->pos().y(), ui->displayWidget->width(), ui->displayWidget->height()));
    animation->setEndValue(QRect(ui->displayWidget->pos().x() + resizeValue, ui->displayWidget->pos().y() + resizeValue, 0, ui->displayWidget->height()));
//    //animation->setEndValue(QRect(pos().x() + width(), pos().y()+height(), 0, 0));
//    animation->setStartValue(100);
//    animation->setEndValue(400);
    animation->setEasingCurve(QEasingCurve::Linear);
    animation->start(QAbstractAnimation::DeleteWhenStopped);
    connect(animation,&QPropertyAnimation::valueChanged,this,&Widget::animationChange);
}
