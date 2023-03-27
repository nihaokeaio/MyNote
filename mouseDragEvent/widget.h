#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QSet>

#include <QPushButton>
#include "panelchild.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    virtual ~Widget();
protected:
    bool eventFilter(QObject* o,QEvent* e);
    void performDrag(QWidget* widget);
private:
    Ui::Widget *ui;

    //\圆盘对象指针
    Panelchild* childPanel;

    QLabel* label;

    //\用于计算拖拽事件是否发生
    QPoint dragStartPostion_;

    //\用于添加需要生成鼠标拖事件的按钮，即可拖拽添加的按钮
    QSet<QPushButton*>link_Btn_;

public slots:
    void slotButtonClicked();
};

#endif // WIDGET_H
