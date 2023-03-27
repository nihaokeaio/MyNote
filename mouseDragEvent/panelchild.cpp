#include "panelchild.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <math.h>


Panelchild::Panelchild(QWidget *parent) : QWidget(parent)
{

    QVBoxLayout *vlay=new QVBoxLayout(this);
    distributId=2;
    aliveObj=distributId;

    scene=new QGraphicsScene(this);
    //scene->setBackgroundBrush(Qt::green);
    //Paintcircle();
    View=new QGraphicsView(scene,this);
    scene->setBackgroundBrush(QBrush(QColor(Qt::gray)));
    //View->setScene(scene);;
    //View->scale(this->width(),this->height());
    View->setAlignment(Qt::AlignCenter);
    View->show();
    vlay->addWidget(View);

}




//startAngle 开始角度
//spanAngle 弧的跨度
//arcHeight 弧的宽度
SubGphItem *Panelchild::gradientArc(qreal radius, qreal startAngle, qreal spanAngle, qreal arcHeight)
{
    SubGphItem* subgphitem=new SubGphItem;


    QRectF rect(-radius,-radius,radius*2,radius*2);
    QPainterPath* path=new QPainterPath;
    path->arcTo(rect,startAngle,spanAngle);

    QPainterPath* subpath=new QPainterPath;
    subpath->addEllipse(rect.adjusted(arcHeight,arcHeight,-arcHeight,-arcHeight));

    qreal midradius=(radius-arcHeight/2);
    const double Pi=3.1415926535;
    qreal posx=midradius*cos((startAngle+spanAngle/2)/180*Pi);
    qreal posy=-midradius*sin((startAngle+spanAngle/2)/180*Pi);
//    QFont font;
//    font.setBold(true);
//    font.setPointSize(12);

//    //path->addText(QPointF(posx,posy),font,QString("Ohayou"));
    subgphitem->setTextPos(QPointF(posx,posy));
    *path-=*subpath;
    //path->closeSubpath();
    subgphitem->setPath(*path);
    subgphitem->setPen(Qt::NoPen);
    subgphitem->setAcceptHoverEvents(true);
    //subgphitem->setFlag(QGraphicsItem::ItemIsMovable);
    this->setAcceptDrops(true);
    return subgphitem;
}



void Panelchild::buildArcPanel()
{
    scene->clear();
    //绘制背景矩形框
    sceneRect=new QGraphicsRectItem;
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::white);
    sceneRect->setPen(pen);
    sceneRect->setRect(QRectF(-CIRCLE_RADIUS_-RECT_GAP_SPACING_,-CIRCLE_RADIUS_-RECT_GAP_SPACING_,(CIRCLE_RADIUS_+RECT_GAP_SPACING_)*2.,(CIRCLE_RADIUS_+RECT_GAP_SPACING_)*2.));
    scene->addItem(sceneRect);
    int numbblock=0;
    //使用QPainterPath来绘制
    for (int i=0;i<distributId;i++) {
        //如果对应的id号已经在被删除对象的集合中，则不做任何操作。
        if(deletedObjId.find(i)!=deletedObjId.end())continue;
        gphitem=new SubGphItem;
        gphitem=gradientArc(CIRCLE_RADIUS_,360./aliveObj*numbblock,360./aliveObj-CIRCLE_GAP_SPACING_,CIRCLE_WIDTH_);
        ++numbblock;
        connect(gphitem,SIGNAL(addGphItem(const QMimeData*,QPushButton*)),this,SLOT(slotAddItem(const QMimeData *,QPushButton*)));
        connect(gphitem,SIGNAL(removeGphItem(const QMimeData*)),this,SLOT(slotRemoveItem(const QMimeData *)));
        connect(gphitem,SIGNAL(exchangeGphItem(const QString,const QMimeData*)),this,SLOT(slotExchangeItem(const QString, const QMimeData* )));
        if(m_list[i]&&m_list[i]->isbuild())
        {
            m_list[i]->setGphItem(gphitem);
            m_list[i]->getGphItem()->setLinkBtn(m_list[i]->getLinkButton());
            //图形看到的面板展示其对象的名字
            m_list[i]->setSubGphItemText(m_list[i]->objectName());
            //真实设置面板名的操作，在对象被构建时已经设置好名字，直接调用再赋值
            m_list[i]->setPanelObjName(m_list[i]->getPanelObjName());
            scene->addItem(gphitem);
            continue;
        }
        SubPanelObj* subPanelObj=new SubPanelObj;
        subPanelObj->setGphItem(gphitem);
        QString strpalname=QString("Pal: %1").arg(i);
        QString strobjname=QString("Obj: %1").arg(i);
        subPanelObj->setObjectName(strobjname);
        subPanelObj->setPanelObjName(strpalname);
        subPanelObj->setbuild(true);
        m_list[i]=subPanelObj;
        subPanelObj->setSubGphItemText(strobjname);
        scene->addItem(gphitem);
    }
    if(m_list[maxNumberBlock]&&m_list[maxNumberBlock]->isbuild())
    {
        scene->addItem(m_list[maxNumberBlock]->getGphItem());
        return;
    }
    //中间圆形面板的初始化
    gphitem=new SubGphItem;
    gphitem=gradientArc(CIRCLE_RADIUS_-CIRCLE_RADIUS_/2,0,360,CIRCLE_RADIUS_-CIRCLE_RADIUS_/2);
    //不应该设有交换功能以及增添功能
    //connect(gphitem,SIGNAL(addGphItem(const QMimeData*)),this,SLOT(slotAddItem(const QMimeData *)));
    connect(gphitem,SIGNAL(removeGphItem(const QMimeData*)),this,SLOT(slotRemoveItem(const QMimeData *)));
    //connect(gphitem,SIGNAL(exchangeGphItem(const QString,const QMimeData*)),this,SLOT(slotExchangeItem(const QString, const QMimeData* )));
    SubPanelObj* subPanelObj=new SubPanelObj;
    subPanelObj->setGphItem(gphitem);
    subPanelObj->setDeleteObj(true);
    QString strobjname=QString("Obj: %1").arg(maxNumberBlock);
    QString strpalname=QString("Pal: %1").arg(maxNumberBlock);
    subPanelObj->setObjectName(strobjname);
    subPanelObj->setPanelObjName(strpalname);
    m_list[maxNumberBlock]=subPanelObj;
    scene->addItem(gphitem);
}

bool Panelchild::addArcPanel(const QMimeData *mimeData,QPushButton* linkBtn)
{
    /*
     * 此处可以设置去重功能，即重复功能的按钮不在添加，简单的便利m_list容器即可
    */
//    qDebug()<<"start->addArcPanel";
    SubPanelObj* subPanelObj=new SubPanelObj;
    //subPanelObj->setLabelText(mimeData->text());
    subPanelObj->setbuild(true);
    subPanelObj->setObjectName(mimeData->text());
    subPanelObj->setPanelObjName(mimeData->text());
    subPanelObj->setLinkButton(linkBtn);
    m_list[distributId]=subPanelObj;
//    qDebug()<<"end->addArcPanel";
    return true;
}

bool Panelchild::removeArcPanel(const QMimeData *mimeData)
{
//    qDebug()<<"start->removeArcPanel";
    QString deletePanel=mimeData->text();
    {
        for(auto m_list_iter=m_list.begin();m_list_iter!=m_list.end();++m_list_iter)
        {
            if(m_list_iter.value()->getPanelObjName()==deletePanel)
            {
                deletedObjId.insert(m_list_iter.key());
                m_list.remove(m_list_iter.key());
//                qDebug()<<"start->removeArcPanel true";
                return true;
            }
        }

    }
//    qDebug()<<"start->removeArcPanel false";
    return false;
}

void Panelchild::paintEvent(QPaintEvent *p)
{
    buildArcPanel();
    QWidget::paintEvent(p);
}

void Panelchild::dropEvent(QDropEvent *event)
{
}

void Panelchild::dragEnterEvent(QDragEnterEvent *event)
{
}


void Panelchild::slotAddItem(const QMimeData *mimeData,QPushButton* linkBtn)
{
    if(addArcPanel(mimeData,linkBtn)&&aliveObj<maxNumberBlock)
    {
       ++distributId;
        ++aliveObj;
    }   
    //qDebug()<<"mimeData->receive:"<<mimeData->text();
    this->update();
}

void Panelchild::slotRemoveItem(const QMimeData *mimeData)
{

    if(removeArcPanel(mimeData)&&aliveObj>0)
    {
        --aliveObj;
    }
    //qDebug()<<"mimeData->receive:"<<mimeData->text();
    this->update();
}

void Panelchild::slotExchangeItem(const QString objName, const QMimeData *mimeData)
{
    SubPanelObj* tempSubPanelObj=new SubPanelObj;
    int leftObj=-1;
    int rightObj=-1;
    for(auto m_list_iter=m_list.begin();m_list_iter!=m_list.end();++m_list_iter)
    {
        if(m_list_iter.value()->getPanelObjName()==objName)
        {

            leftObj=m_list_iter.key();
            qDebug()<<"leftObj : "<<leftObj;

        }
        if(m_list_iter.value()->getPanelObjName()==mimeData->text())
        {
            rightObj=m_list_iter.key();
            qDebug()<<"rightObj : "<<rightObj;
        }
        if(leftObj!=-1&&rightObj!=-1)
            break;
    }
    if(leftObj!=-1&&rightObj!=-1)
    {
        tempSubPanelObj=m_list[leftObj];
        m_list[leftObj]=m_list[rightObj];
        m_list[rightObj]=tempSubPanelObj;
    }

    this->update();
}
