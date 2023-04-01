#ifndef MYVIEW_H
#define MYVIEW_H
#include <QGraphicsView>
#include <QMouseEvent>

class MyGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit MyGraphicsView(QGraphicsView* parent = nullptr);
protected:
    void mouseMoveEvent(QMouseEvent* event) override;
};

#endif // MYVIEW_H
