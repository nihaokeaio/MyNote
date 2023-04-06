#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QtCharts>
#include <QList>
#include <QPropertyAnimation>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(int myValue READ getReadMyValue WRITE setReadMyValue USER false)
public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

    int getReadMyValue();
    void setReadMyValue(int val);

public slots:
    void animationChange(QVariant value);
    void slotStartcharts();
    void slotStartAnimation();
private:
    Ui::Widget *ui;
    QPushButton* startAnimation;
    QPushButton* startcharts;
    QChart* chart;
    QPropertyAnimation* animation;
    QList<QPointF>datalist;


    int time=0;
    int m_value;
};

#endif // WIDGET_H
