#ifndef DRAGBUTTON_H
#define DRAGBUTTON_H
#include <QPushButton>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QMouseEvent>
#include <QWidget>

class Dragbutton : public QPushButton
{
public:
    explicit Dragbutton(QWidget *parent = nullptr);

    virtual void dragEnterEvent(QDragEnterEvent *event);
    virtual void dragMoveEvent(QDragMoveEvent *event);
//    virtual void dragLeaveEvent(QDragLeaveEvent *event);
    //virtual void dropEvent(QDropEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    void performDrag();
private:
    QPoint dragStartPostion_;
};

#endif // DRAGBUTTON_H
