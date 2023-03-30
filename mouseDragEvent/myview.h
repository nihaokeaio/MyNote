#ifndef MYVIEW_H
#define MYVIEW_H
#include <QGraphicsView>
#include <QMouseEvent>

class MyView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit MyView(QGraphicsView* parent = nullptr);
protected:
    void mouseMoveEvent(QMouseEvent* event) override;
};

#endif // MYVIEW_H
