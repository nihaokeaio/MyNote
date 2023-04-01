#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include "RoulettePanel.h"

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
   

private:
    Ui::Widget *ui;

    QLabel* label;

    RoulettePanel* roulettePanel;

public slots:
    void slotButtonClicked();
};

#endif // WIDGET_H
