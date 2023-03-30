#include "myview.h"

#include <QDebug>
#include <QGraphicsPathItem>

MyView::MyView(QGraphicsView* parent) : QGraphicsView(parent)
{

}

void MyView::mouseMoveEvent(QMouseEvent* event)
{
    //qDebug() << "MyView pos:" << event->pos();
    this->setFocus();
    /*QGraphicsPathItem* gph = static_cast<QGraphicsPathItem*>(this->itemAt(event->pos()));
    if (gph) {
        gph->setFocus();
    }*/
    this->scene()->setFocus();
    QGraphicsView::mouseMoveEvent(event);
}
