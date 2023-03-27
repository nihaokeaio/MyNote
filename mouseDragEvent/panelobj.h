#ifndef PANELOBJ_H
#define PANELOBJ_H

#include <QObject>
#include <QPainterPath>
#include <QMouseEvent>
#include <QDebug>
#include "subgphitem.h"


class SubPanelObj : public QObject
{
    Q_OBJECT
public:
    explicit SubPanelObj(QObject *parent = nullptr);

    //关于面板的操作
    void setGphItem(SubGphItem *subGphItem);
    SubGphItem* getGphItem();
    //用于判断该对象是否已经被构建
    bool isbuild();
    void setbuild(bool flag);
    //设置label显示
    void setLabelText(QString str);
    void setSubGphItemText(QString str);
    //判断该对象面板是否为删除面板（即中心的圆圈区域）
    bool isDeleteObj();
    void setDeleteObj(bool flag);
    //设置_获得子面板的对象名称
    void setPanelObjName(QString name);
    QString getPanelObjName();

    bool isExchangeObj();
    void setExchangeObj(bool flag);

    void setLinkButton(QPushButton *linkBtn);
    QPushButton* getLinkButton();

    QString getLabelText();

    void manualInstallFilter();



public:
    bool eventFilter(QObject* obj,QEvent* event) override;
signals:


public slots:

private:
     SubGphItem* subGphItem_;
     bool isbuild_;
     QString objLabel_;
     bool isdeletePanel_;
     bool isExchangePanel_;
     QString PanelObjName_;
     QPushButton* subBtn_;
};

#endif // PANELOBJ_H
