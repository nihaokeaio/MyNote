#ifndef ROULETTE_H
#define ROULETTE_H

#include <QWidget>
#include <QSet>
#include <QPushButton>
#include <QKeyEvent>

#include "ScenePanel.h"

class RoulettePanel : public QWidget
{
    Q_OBJECT
public:
    explicit RoulettePanel(QWidget* parent = nullptr);

    //设置可放入的按钮对象
    void addLinkButton(QPushButton* button);

    //设置初始圆盘的大小，outDiameter-外径，inDiameter-内径，gapSpacing-间隙
    void adjustRouletteSize(qreal outDiameter, qreal inDiameter,qreal gapSpacing);

    //设置初始放入轮盘的按钮
    void setInitButtons(QVector<QPushButton*>buttons);

protected:
    bool eventFilter(QObject* o, QEvent* e);
    void performDrag(QWidget* widget);

    //键盘事件
    void keyPressEvent(QKeyEvent* event) override; //键盘按下事件
    void keyReleaseEvent(QKeyEvent* event) override; //键盘松开事件

private:
    //\圆盘对象指针
    ScenePanel* childPanel;

    //\用于计算拖拽事件是否发生
    QPoint dragStartPostion_;

    //\用于添加需要生成鼠标拖事件的按钮，即可拖拽添加的按钮
    QSet<QPushButton*>linkBtnSet_;

public slots:
    
};

#endif // WIDGET_H
