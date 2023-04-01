#ifndef PANELOBJ_H
#define PANELOBJ_H

#include <QObject>
#include <QPainterPath>
#include <QMouseEvent>
#include <QDebug>
#include "SubGphItem.h"


class SubGphObj : public QObject
{
    Q_OBJECT
public:
    explicit SubGphObj(QObject *parent = nullptr);

    //关于面板的操作
    void setGphItem(SubGphItem *subGphItem);
    SubGphItem* getGphItem();

    //用于判断该对象是否已经被构建
    bool isbuild();
    void setbuild(bool flag);

    //设置面板label显示
    void setSubGphItemText(QString str);

    //判断该对象面板是否为删除面板（即中心的圆圈区域）
    bool isDeleteObj();
    void setDeleteObj(bool flag);

    //设置_获得面板指针指向的对象名称
    void setSubGphItemName(QString name);
    QString getSubGphObjName();

    //设置_获得面板指针指向的对象是否为可交换的面板
    bool isExchangeObj();
    void setExchangeObj(bool flag);

    //设置_获得面板指针指向的对象中按钮链接的按钮
    void setLinkButton(QPushButton *linkBtn);
    QPushButton* getLinkButton();

public:
    //bool eventFilter(QObject* obj,QEvent* event) override;
signals:


public slots:

private:
    //\面板指针，用来指向对应的面板（其本身每次都需要重绘）
     SubGphItem* subGphItem_;

     //\用来判断该面板对象是否已经生成，每次重绘只需重绘面板即可，不需要整个对象都重构
     bool isBuild_;

     //\无意义
     QString objLabel_;

     //\记录面板的对象名，重绘只是重绘面板，尽可能保持其他不变
     QString subGphObjName_;

     //\记录面板的链接的按钮
     QPushButton* subBtn_;
};

#endif // PANELOBJ_H
