#include "mygraphicsscene.h"
#include <QDebug>

MyGraphicsScene::MyGraphicsScene(QGraphicsScene* parent) : QGraphicsScene(parent)
{

}

void MyGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    //qDebug() << "pos:" << event->pos();
    QTransform trans;
    //qDebug() << "MyGraphicsScene:" << event->scenePos();
    //qDebug()<<"MyGraphicsScene:"<<this->itemAt(event->scenePos(),trans);
    QGraphicsPathItem* gph = static_cast<QGraphicsPathItem*>(this->itemAt(event->scenePos(), trans));
    if (gph)
    {
        gph->setFocus();
    }
    

    QGraphicsScene::mouseMoveEvent(event);
}

