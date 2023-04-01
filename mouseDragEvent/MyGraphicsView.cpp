#include "MyGraphicsView.h"

#include <QDebug>
#include <QGraphicsPathItem>

MyGraphicsView::MyGraphicsView(QGraphicsView* parent) : QGraphicsView(parent)
{

}

void MyGraphicsView::mouseMoveEvent(QMouseEvent* event)
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
