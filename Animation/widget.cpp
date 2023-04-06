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
    //ui->displayWidget->setBackgroundBrush(QBrush(Qt::red));
    connect(startcharts,&QPushButton::clicked,this,&Widget::slotStartcharts);
    connect(startAnimation,&QPushButton::clicked,this,&Widget::slotStartAnimation);


    QVBoxLayout* vlay=new QVBoxLayout(this);
    vlay->addLayout(hlay);
    vlay->addWidget(ui->displayWidget);
    vlay->addWidget(ui->chartsWidget);
#if 1
    QPropertyAnimation* pAnimationAlpha=new QPropertyAnimation();
    pAnimationAlpha->setTargetObject(this);
    pAnimationAlpha->setPropertyName("myValue");
    pAnimationAlpha->setDuration(1000);
    pAnimationAlpha->setKeyValueAt(0, 255);
    pAnimationAlpha->setKeyValueAt(0.5, 100);
    pAnimationAlpha->setKeyValueAt(1, 255);
    pAnimationAlpha->setLoopCount(-1);  //永远运行，直到stop
    connect(startcharts, SIGNAL(clicked(bool)), pAnimationAlpha, SLOT(start()));
#endif

#if 1
    QGraphicsOpacityEffect* my_widgetProcessOpacity;
    my_widgetProcessOpacity=new QGraphicsOpacityEffect(startcharts);
    my_widgetProcessOpacity->setOpacity(0);
    startcharts->setGraphicsEffect(my_widgetProcessOpacity);

    QPropertyAnimation* pAnimation=new QPropertyAnimation(my_widgetProcessOpacity,"opacity",startcharts);
    pAnimation->setDuration(1000);
    pAnimation->setStartValue(0);
    pAnimation->setEndValue(1);
    pAnimation->setLoopCount(-1);  //永远运行，直到stop
    pAnimation->setEasingCurve(QEasingCurve::InOutQuart);
    //animation->start(QAbstractAnimation::DeleteWhenStopped);
    connect(startAnimation, SIGNAL(clicked(bool)), pAnimation, SLOT(start()));
#endif

    QSequentialAnimationGroup* pSequenAno = new QSequentialAnimationGroup(this);

    //透明度从0变为1，显示出来；

    pSequenAno->addAnimation(pAnimation);

    //暂停一秒

    QPauseAnimation *pPauseAnimation = new QPauseAnimation(this);

    pPauseAnimation->setDuration(1000);

    //再向上移动

    pSequenAno->addAnimation(pAnimationAlpha);

    pSequenAno->start(QAbstractAnimation::DeleteWhenStopped);

}

Widget::~Widget()
{
    delete ui;
}

int Widget::getReadMyValue()
{
    return m_value;
}

void Widget::setReadMyValue(int val)
{
    m_value=val;
    QString strQSS = QString("color: rgb(0, 160, 230); background-color: rgba(10, 160, 105, %1);").arg(m_value);
    this->setStyleSheet(strQSS);
}

void Widget::animationChange(QVariant value)
{
    ++time;
    datalist.append(QPointF(time,value.toRect().x()));
}

void Widget::slotStartcharts()
{
     ui->chartsWidget->clearMask();
    QSplineSeries* series=new QSplineSeries;
    series->append(datalist);

    chart=new QChart;
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->axes(Qt::Vertical).first()->setRange(0,1000);
    chart->axes(Qt::Horizontal).first()->setRange(0,time);
    ui->chartsWidget->setChart(chart);
    datalist.clear();
    time=0;
}

void Widget::slotStartAnimation()
{
    static int resizeValue{ui->displayWidget->width()};

    animation = new QPropertyAnimation(ui->displayWidget, "geometry");
    animation->setDuration(500);
    qDebug()<<QString("%1,%2,%3,%4").arg(ui->displayWidget->pos().x()).arg(ui->displayWidget->pos().y()).arg(ui->displayWidget->width()).arg(ui->displayWidget->height());
    animation->setStartValue(QRect(ui->displayWidget->pos().x(), ui->displayWidget->pos().y(), ui->displayWidget->width(), ui->displayWidget->height()));

    qDebug()<<ui->displayWidget->pos().x() + resizeValue;
    qDebug()<<ui->displayWidget->width() - resizeValue;
    int width_h=ui->displayWidget->width() - resizeValue;
    qDebug()<<ui->displayWidget->width();
    qDebug()<<+ resizeValue;

    animation->setEndValue(QRect(ui->displayWidget->pos().x() + resizeValue, ui->displayWidget->pos().y(), width_h, ui->displayWidget->height()));
//    //animation->setEndValue(QRect(pos().x() + width(), pos().y()+height(), 0, 0));
    resizeValue = -resizeValue;
//    animation->setStartValue(100);
//    animation->setEndValue(400);
    animation->setEasingCurve(QEasingCurve::InOutQuart);
    animation->start(QAbstractAnimation::DeleteWhenStopped);
    connect(animation,&QPropertyAnimation::valueChanged,this,&Widget::animationChange);










}
