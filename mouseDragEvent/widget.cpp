#include "widget.h"
#include "ui_widget.h"
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QLabel>
#include <QDrag>



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setAttribute(Qt::WA_TranslucentBackground);
    this->resize(600, 800);
    
    QVBoxLayout* vlayout=new QVBoxLayout(this);
   
    QPushButton* button=new QPushButton;
    button->setObjectName("NHKAO");
    button->setText("NHKAO");
    button->setCheckable(true);
    button->setChecked(false);
    this->setTabletTracking(true);
    childPanel=new Panelchild;

    button->installEventFilter(this);

    label= new QLabel;
    label->setText("yaho");
    label->setVisible(true);
    label->setEnabled(true);
    vlayout->addWidget(childPanel);
    vlayout->addWidget(button);
    vlayout->addWidget(label);

    link_Btn_.insert(button);

    childPanel->setVisible(true);
    
    connect(button,SIGNAL(clicked()),this,SLOT(slotButtonClicked()));
}

Widget::~Widget()
{
    delete ui;
}

void Widget::performDrag(QWidget* widget)
{
    QMimeData *mimeData = new QMimeData;
    mimeData->setText(widget->objectName());
    qDebug()<<("%f",mimeData);
    QDrag *drag = new QDrag(widget);
    drag->setMimeData(mimeData);
    //drag->setPixmap(QPixmap(":/images/person.png"));
    if (drag->exec(Qt::MoveAction) == Qt::MoveAction);

}

bool Widget::eventFilter(QObject *o, QEvent *e)
{
    QPushButton* eventBtn=qobject_cast<QPushButton*>(o);
    if(link_Btn_.find(eventBtn)!=link_Btn_.end())
    {
        if (e->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent* event = static_cast<QMouseEvent*>(e);
            if (event->button()==Qt::LeftButton)
            {
                dragStartPostion_ = event->pos();
                Widget::mousePressEvent(event);
            }
        }

        if(e->type()==QEvent::MouseMove)
        {
            QMouseEvent* event=static_cast<QMouseEvent*>(e);
            if (event->buttons() & Qt::LeftButton) {
                    int distance = (event->pos() - dragStartPostion_).manhattanLength();
                    if (distance >= QApplication::startDragDistance())
                    {
                        performDrag(eventBtn);
                        eventBtn->clearFocus();
                    }
                }
                Widget::mouseMoveEvent(event);
        }
    }

    return QWidget::eventFilter(o,e);
}

void Widget::slotButtonClicked()
{
    this->label->setText("receive");
    qDebug()<<label->text();
}


void Widget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Alt)
    {
        qDebug() << "Key_Alt keyPressEvent before " << this->focusWidget();
        childPanel->setVisible(true);
        childPanel->setFocus();
        qDebug() << "Key_Alt keyPressEvent after " << this->focusWidget();
    }

    if (event->key() == Qt::Key_C)
    {
        qDebug() << "Key_C keyPressEvent before " << this->focusWidget();
    }
}

void Widget::keyReleaseEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Alt)
    {
        childPanel->setVisible(true);
    }
}






