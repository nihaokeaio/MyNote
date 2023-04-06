#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();
    void moveUp(QWidget* widget);
    void fadeAway(QWidget* widget);
private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
