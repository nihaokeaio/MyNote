#include "mygraphicsscene.h"
#include <QDebug>

MyGraphicsScene::MyGraphicsScene(QGraphicsScene *parent): QGraphicsScene(parent)
{

}

void MyGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    qDebug()<<"pos:"<<event->pos();
    QGraphicsScene::mouseMoveEvent(event);
}

