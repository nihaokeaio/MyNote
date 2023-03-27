#ifndef SUBGPHITEM_H
#define SUBGPHITEM_H

#include <QGraphicsPathItem>
#include <QObject>
#include <QPaintEvent>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QHoverEvent>
#include <QGraphicsSimpleTextItem>
#include <QMimeData>
#include <QPushButton>

class SubGphItem :public QObject, public QGraphicsPathItem
{
    Q_OBJECT
    //Q_INTERFACES(QGraphicsPathItem)
public:
    explicit SubGphItem(QGraphicsPathItem *parent = nullptr);

    //\设置_得到文本的位置
    void setTextPos(const QPointF& pos);
    QPointF& getTextPos();

    //设置_是否为用于删除的面板
    bool isDeletePanel();
    void setDeletePanel(bool flag);

    //设置_是否为用于交换的面板
    bool isExchangePanel();
    void setExchangePanel(bool flag);

    //设置_得到link按钮
    void setLinkBtn(QPushButton* linkBtn);
    QPushButton* getLinkBtn();

    //\拖事件发生的自定义行为
    void performDrag();
signals:
    void addGphItem(const QMimeData* mimeData,QPushButton*);

    void removeGphItem(const QMimeData* mimeData);

    void exchangeGphItem(const QString objName,const QMimeData* mimeData);
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

//    void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;

    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;


    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;


    void dropEvent(QGraphicsSceneDragDropEvent *event) override;

    virtual void dragEnterEvent(QGraphicsSceneDragDropEvent *event) override;

public:
    //\文本图样，最终显示在圆盘上的文本对象
      QGraphicsSimpleTextItem* gphtext;
private:

      //\主要用于描述文本应该放置的位置
      QPointF textPos_;

      //\判断该面板是否为可删除的面板，即中间的圆
      bool isDeletePanel_;

      //\判断该面板是否为可交换的，其实我觉得没什么用，应该可以优化掉
      bool isExchangePanel_;

      //鼠标拖动距离，判断是否为误触动
      QPointF dragStartPostion_;

      //用于排除拖和放对象都是自身的情况
      bool isDraging_;

      //\全局的一个按钮指针，用来承接被拖拽对象的按钮功能，发出信号用
      QPushButton* linkBtn_;

public slots:

};

#endif // SUBGPHITEM_H
