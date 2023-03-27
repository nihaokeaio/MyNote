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

    //bool sceneEvent(QEvent *event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

//    void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;

    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;


    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;


    void dropEvent(QGraphicsSceneDragDropEvent *event);

    virtual void dragEnterEvent(QGraphicsSceneDragDropEvent *event);

    void setTextPos(const QPointF& pos);

    //是否为用于删除的面板
    bool isDeletePanel();

    void setDeletePanel(bool flag);

    bool isExchangePanel();

    void setExchangePanel(bool flag);

    void setLinkBtn(QPushButton* linkBtn);

    QPushButton* getLinkBtn();


    void performDrag();
signals:
    void addGphItem(const QMimeData* mimeData,QPushButton*);

    void removeGphItem(const QMimeData* mimeData);

    void exchangeGphItem(const QString objName,const QMimeData* mimeData);
public:
//    SubGphItem* subgphitem;

//    int radius_;
//    qreal startAngle_;
//    qreal spanAngle_;
//    int arcHeight_;
//    QRgb color_;
      QGraphicsSimpleTextItem* gphtext;
private:
      QPointF textPos_;
      bool isDeletePanel_;
      bool isExchangePanel_;
      //鼠标拖动距离，判断是否为误触动
      QPointF dragStartPostion_;
      //用于排除拖和放对象都是自身的情况
      bool isDraging_;
      QPushButton* linkBtn_;


public slots:
};

#endif // SUBGPHITEM_H
