#include "widget.h"
#include "ui_widget.h"
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QDrag>



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    //this->setWindowFlags(Qt::FramelessWindowHint);
    //this->setAttribute(Qt::WA_TranslucentBackground);
    this->resize(400, 600);
    this->setTabletTracking(true);
    this->move(1300, 100);

    QVBoxLayout* vlayout=new QVBoxLayout(this);
   
    QPushButton* button=new QPushButton;
    button->setObjectName("NHKAO");
    button->setText("NHKAO");
    button->setCheckable(true);
    button->setChecked(false);
    

    label= new QLabel;
    label->setText("ABCD");
    label->setVisible(true);
    label->setEnabled(true);

    roulettePanel = new RoulettePanel;

    vlayout->addWidget(button);
    vlayout->addWidget(label);

    roulettePanel->addLinkButton(button);

    QVector<QPushButton*>buttons;
    buttons.push_back(button);

    roulettePanel->setInitButtons(buttons);

    roulettePanel->setVisible(true);

    roulettePanel->adjustRouletteSize(150, 60, 1);
    
    connect(button,SIGNAL(clicked()),this,SLOT(slotButtonClicked()));
}

Widget::~Widget()
{
    delete ui;
}

void Widget::slotButtonClicked()
{
    this->label->setText("receive");
    qDebug()<<label->text();
}








