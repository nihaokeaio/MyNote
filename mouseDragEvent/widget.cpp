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
    QVBoxLayout* vlayout=new QVBoxLayout(this);
    QPushButton* addButton=new QPushButton;
    addButton->setText("addButton");
    addButton->setObjectName("addButton");
    addButton->setCheckable(true);
    addButton->setChecked(false);

    QPushButton* subButton=new QPushButton;
    subButton->setText("subButton");
    subButton->setObjectName("subButton");
    subButton->setCheckable(true);
    subButton->setChecked(false);


    vlayout->addWidget(addButton);
    vlayout->addWidget(subButton);

    QPushButton* button=new QPushButton;
    button->setObjectName("NHKAO");
    button->setText("NHKAO");
    button->setCheckable(true);
    button->setChecked(false);
    this->setTabletTracking(true);
    childPanel=new Panelchild;

    button->installEventFilter(this);
    addButton->installEventFilter(this);
    subButton->installEventFilter(this);

    label= new QLabel;
    label->setText("yaho");
    label->setVisible(true);
    label->setEnabled(true);
    vlayout->addWidget(childPanel);
    vlayout->addWidget(button);
    vlayout->addWidget(label);

    link_Btn_.insert(button);
    link_Btn_.insert(addButton);
    link_Btn_.insert(subButton);
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
                        performDrag(eventBtn);
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







