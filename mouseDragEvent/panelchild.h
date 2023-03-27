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
    SubGphItem* gradientArc(int radius,qreal startAngle,qreal spanAngle,
                     int arcHeight);
    void buildArcPanel();

    bool addArcPanel(const QMimeData *mimeData,QPushButton* linkBtn);

    bool removeArcPanel();

    bool removeArcPanel(const QMimeData *mimeData);

    bool insertArcPanel(SubGphItem* preitem);

    void paintEvent( QPaintEvent * p);
    void dropEvent(QDropEvent *event);
    virtual void dragEnterEvent(QDragEnterEvent *event);

signals:

public slots:
    void slotAddItem();
    void slotSubItem();

    void slotAddItem(const QMimeData* mimeData,QPushButton*);
    void slotRemoveItem(const QMimeData* mimeData);
    void slotExchangeItem(QString const objName,const QMimeData* mimeData);
private:
    int aliveObj;
    const int maxNumberBlock=20;
    QGraphicsRectItem* sceneRect;
    QGraphicsScene* scene;
    QGraphicsView* View;
    SubGphItem* gphitem;
    QMap<int,SubPanelObj*> m_list;
    QMap<QString,SubPanelObj*> m_obj_list;
    //friend class PanelObj;
    QSet<int>deletedObjId;
    int distributId;

};

#endif // PANELCHILD_H
