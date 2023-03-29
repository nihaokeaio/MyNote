#ifndef MYGRAPHICSSCENE_H
#define MYGRAPHICSSCENE_H
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

class MyGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit MyGraphicsScene(QGraphicsScene *parent = nullptr);
protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
};

#endif // MYGRAPHICSSCENE_H
