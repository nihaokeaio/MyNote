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


    Panelchild* childPanel;
    QLabel* label;
    QPoint dragStartPostion_;
    QSet<QPushButton*>link_Btn_;

public slots:
    void slotButtonClicked();
};

#endif // WIDGET_H
