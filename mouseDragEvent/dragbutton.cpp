#include "dragbutton.h"
#include <QDebug>
#include <QApplication>
#include <QMimeData>
#include <QDrag>

class MyMimeData:public QMimeData
{

};


Dragbutton::Dragbutton(QWidget *parent) : QPushButton(parent)
{
    //this->setAcceptDrops(true);
}

void Dragbutton::dragEnterEvent(QDragEnterEvent *event)
{
     //event->acceptProposedAction();
}


void Dragbutton::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
            int distance = (event->pos() - dragStartPostion_).manhattanLength();
            if (distance >= QApplication::startDragDistance())
                performDrag();
        }
        QPushButton::mouseMoveEvent(event);
}


void Dragbutton::performDrag()
{
    QMimeData *mimeData = new QMimeData;
    mimeData->setText(this->objectName());
    qDebug()<<("%f",mimeData);
    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    //drag->setPixmap(QPixmap(":/images/person.png"));
    if (drag->exec(Qt::MoveAction) == Qt::MoveAction);

}

void Dragbutton::dragMoveEvent(QDragMoveEvent *event)
{
//    QObject *source=event->source();
//    if (source && source != this) {
//        event->setDropAction(Qt::MoveAction);
//        event->accept();
    //}
}

void Dragbutton::mousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        dragStartPostion_ = event->pos();
    }
    QPushButton::mousePressEvent(event);
}
