#ifndef PANELCHILD_H
#define PANELCHILD_H

#include <QWidget>
#include <QPainter>
#include <QRadialGradient>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMap>
#include <QDropEvent>

#include "panelobj.h"
#include "dragbutton.h"
#include "mygraphicsscene.h"
#include "myview.h"
#include <QMimeData>
#include <QSet>

//定义的圆盘半径
const qreal CIRCLE_RADIUS_=150;
//定义的圆盘宽度
const qreal CIRCLE_WIDTH_=60;
//圆盘缝隙间距
const qreal CIRCLE_GAP_SPACING_=1;
//矩形缝隙间距
const qreal RECT_GAP_SPACING_=5;

namespace Ui {
class Widget;
}

class Panelchild : public QWidget
{
    Q_OBJECT
public:
    explicit Panelchild(QWidget *parent = nullptr);
    SubGphItem* gradientArc(qreal radius,qreal startAngle,qreal spanAngle,
                     qreal arcHeight);
    void buildArcPanel();

    bool addArcPanel(const QMimeData *mimeData,QPushButton* linkBtn);

    bool removeArcPanel();

    bool removeArcPanel(const QMimeData *mimeData);

    bool insertArcPanel(SubGphItem* preitem);

protected:
    void paintEvent( QPaintEvent * p);
    void dropEvent(QDropEvent *event);
    virtual void dragEnterEvent(QDragEnterEvent *event);

    

signals:

public slots:

    void slotAddItem(const QMimeData* mimeData,QPushButton*);
    void slotRemoveItem(const QMimeData* mimeData);
    void slotExchangeItem(QString const objName,const QMimeData* mimeData);
    //void slotIsSelected(SubGphItem* item);
private:
    //\活着的对象，设置了初始值为2
    int aliveObj;

    //\设置的最大的按钮对象，暂时设置为20
    const int maxNumberBlock=20;

    //\设置的矩形画板，暂时设置为（-155，-155,310,310）
    QGraphicsRectItem* sceneRect;

    //\画板屏幕
    QGraphicsScene* scene;

    //\视图
    QGraphicsView* View;

    //\弧形画板，每次变化都需重构
    SubGphItem* gphitem;

    //\用来保存已经添加的按钮画板对象
    QMap<int,SubPanelObj*> m_list;

    //\保存已经被移除的对象，
    QSet<int>deletedObjId;

    //\标识符，每次新分配一个对象机会得到一个新的标识符，不递减
    int distributId;

    SubPanelObj* backgroundSubPanel;

};

#endif // PANELCHILD_H
