#include "ScenePanel.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QPalette>
#include <math.h>


ScenePanel::ScenePanel(QWidget *parent) : QWidget(parent)
{ 
    QVBoxLayout *vlay=new QVBoxLayout(this);
    distributId=2;
    aliveObj=distributId;

    backgroundSubPanel = new SubGphObj; 
    backgroundSubPanel->setbuild(true);
    QString strobjname = QString("Obj: %1").arg(maxNumberBlock);
    backgroundSubPanel->setObjectName(strobjname);
    linkGphObj_[maxNumberBlock] = backgroundSubPanel;

    scene = new MyGraphicsScene;
    scene->setParent(this);

    View = new MyGraphicsView;
    View->setScene(scene);
    //View->setStyleSheet("background:transparent");
    QPalette pal=View->palette();
    pal.setColor(QPalette::Window,Qt::transparent);
    View->setPalette(pal);

    View->setMouseTracking(true);

    View->setAlignment(Qt::AlignCenter);

    View->setCacheMode(QGraphicsView::CacheBackground);


    View->show();
    vlay->addWidget(View);
    this->update();

}




//startAngle 开始角度
//spanAngle 弧的跨度
//arcHeight 弧的宽度
SubGphItem * ScenePanel::gradientArc(qreal radius, qreal startAngle, qreal spanAngle, qreal arcHeight)
{
    SubGphItem* subgphitem=new SubGphItem;


    QRectF rect(-radius,-radius,radius*2,radius*2);
    QPainterPath path=QPainterPath();
    path.arcTo(rect,startAngle,spanAngle);

    QPainterPath subpath=QPainterPath();
    subpath.addEllipse(rect.adjusted(arcHeight,arcHeight,-arcHeight,-arcHeight));

    qreal midradius=(radius-arcHeight/2);
    const double Pi=3.1415926535;
    qreal posx=midradius*cos((startAngle+spanAngle/2)/180*Pi);
    qreal posy=-midradius*sin((startAngle+spanAngle/2)/180*Pi);
//    QFont font;
//    font.setBold(true);
//    font.setPointSize(12);

//    //path->addText(QPointF(posx,posy),font,QString("Ohayou"));
    subgphitem->setTextPos(QPointF(posx,posy));
    path-=subpath;
    //path->closeSubpath();
    subgphitem->setPath(path);
    subgphitem->setPen(Qt::NoPen);
    subgphitem->setAcceptHoverEvents(true);
    //subgphitem->setFlag(QGraphicsItem::ItemIsMovable);
    this->setAcceptDrops(true);
    return subgphitem;
}



void ScenePanel::buildArcPanel()
{
    scene->clear();
    
    //绘制背景矩形框
    //sceneRect=new QGraphicsRectItem;
    //QPen pen;
    //pen.setWidth(1);
    //pen.setColor(Qt::white);
    ///*sceneRect->setOpacity(0.1);*/
    //sceneRect->setPen(pen);
    //sceneRect->setRect(QRectF(-CIRCLE_RADIUS_-RECT_GAP_SPACING_,-CIRCLE_RADIUS_-RECT_GAP_SPACING_,(CIRCLE_RADIUS_+RECT_GAP_SPACING_)*2.,(CIRCLE_RADIUS_+RECT_GAP_SPACING_)*2.));
    //scene->addItem(sceneRect);

    int numbblock=0;
    //使用QPainterPath来绘制
    for (int i=0;i<distributId;i++) {
        //如果对应的id号已经在被删除对象的集合中，则不做任何操作。
        if(deletedObjId.find(i)!=deletedObjId.end())continue;
        //gphItem=new SubGphItem;
        gphItem=gradientArc(CIRCLE_RADIUS_,360./aliveObj*numbblock,360./aliveObj-CIRCLE_GAP_SPACING_,CIRCLE_WIDTH_);
        ++numbblock;
        connect(gphItem,SIGNAL(addGphItem(const QMimeData*,QPushButton*)),this,SLOT(slotAddItem(const QMimeData *,QPushButton*)));
        connect(gphItem,SIGNAL(removeGphItem(const QMimeData*)),this,SLOT(slotRemoveItem(const QMimeData *)));
        connect(gphItem,SIGNAL(exchangeGphItem(const QString,const QMimeData*)),this,SLOT(slotExchangeItem(const QString, const QMimeData* )));
        if(linkGphObj_[i]&&linkGphObj_[i]->isbuild())
        {
            linkGphObj_[i]->setGphItem(gphItem);
            linkGphObj_[i]->getGphItem()->setLinkBtn(linkGphObj_[i]->getLinkButton());
            //图形看到的面板展示其对象的名字
            linkGphObj_[i]->setSubGphItemText(linkGphObj_[i]->objectName());
            //真实设置面板名的操作，在对象被构建时已经设置好名字，直接调用再赋值
            linkGphObj_[i]->setSubGphItemName(linkGphObj_[i]->getSubGphObjName());
            scene->addItem(gphItem);
            continue;
        }
        SubGphObj* subGphObj=new SubGphObj;
        subGphObj->setGphItem(gphItem);
        QString strpalname=QString("Pal: %1").arg(i);
        QString strobjname=QString("Obj: %1").arg(i);
        subGphObj->setObjectName(strobjname);
        subGphObj->setSubGphItemName(strpalname);
        subGphObj->setbuild(true);
        linkGphObj_[i]= subGphObj;
        subGphObj->setSubGphItemText(strobjname);
        scene->addItem(gphItem);
    } 
    //中间圆形面板的初始化
    //gphItem=new SubGphItem;
    gphItem=gradientArc(CIRCLE_RADIUS_-CIRCLE_RADIUS_/2,0,360,CIRCLE_RADIUS_-CIRCLE_RADIUS_/2);
    //不应该设有交换功能以及增添功能
    //connect(gphItem,SIGNAL(addGphItem(const QMimeData*)),this,SLOT(slotAddItem(const QMimeData *)));
    connect(gphItem,SIGNAL(removeGphItem(const QMimeData*)),this,SLOT(slotRemoveItem(const QMimeData *)));
    //connect(gphItem,SIGNAL(exchangeGphItem(const QString,const QMimeData*)),this,SLOT(slotExchangeItem(const QString, const QMimeData* )));
    backgroundSubPanel->setGphItem(gphItem);
    QString strpalname = QString("Pal: %1").arg(maxNumberBlock);
    backgroundSubPanel->setSubGphItemName(strpalname);
    backgroundSubPanel->setDeleteObj(true);
    backgroundSubPanel->setExchangeObj(false);

    scene->addItem(gphItem);
}

bool ScenePanel::addArcPanel(const QMimeData *mimeData,QPushButton* linkBtn)
{
    for (auto m_list_iter = linkGphObj_.begin(); m_list_iter != linkGphObj_.end(); ++m_list_iter)
    {
        if (m_list_iter.value()->getSubGphObjName() == mimeData->text())
        {
            qDebug() << "The button " << mimeData->text() << "has been created.";
            return false;
        }
    }
//    qDebug()<<"start->addArcPanel";
    SubGphObj* subGphObj =new SubGphObj;
    //SubGphObj->setLabelText(mimeData->text());
    subGphObj->setbuild(true);
    subGphObj->setObjectName(mimeData->text());
    subGphObj->setSubGphItemName(mimeData->text());
    subGphObj->setLinkButton(linkBtn);
    linkGphObj_[distributId]= subGphObj;
//    qDebug()<<"end->addArcPanel";
    return true;
}

bool ScenePanel::removeArcPanel(const QMimeData *mimeData)
{
//    qDebug()<<"start->removeArcPanel";
    QString deletePanel=mimeData->text();
    {
        for(auto m_list_iter=linkGphObj_.begin();m_list_iter!=linkGphObj_.end();++m_list_iter)
        {
            if(m_list_iter.value()->getSubGphObjName()==deletePanel)
            {
                deletedObjId.insert(m_list_iter.key());
                linkGphObj_.remove(m_list_iter.key());
//                qDebug()<<"start->removeArcPanel true";
                return true;
            }
        }

    }
//    qDebug()<<"start->removeArcPanel false";
    return false;
}

void ScenePanel::adjustRouletteSize(qreal outDiameter, qreal inDiameter,qreal gapSpacing)
{
    if (outDiameter > inDiameter)
    {
        CIRCLE_RADIUS_ = outDiameter;
    }
    if (inDiameter > 0 && outDiameter > inDiameter)
    {
        CIRCLE_WIDTH_ = inDiameter;
    }
    if (gapSpacing > 0)
    {
        CIRCLE_GAP_SPACING_ = gapSpacing;
    }
    
}

void ScenePanel::paintEvent(QPaintEvent *p)
{
    //qDebug() << "ScenePanel paintEvent start";
    buildArcPanel();
    QWidget::paintEvent(p);
    //qDebug() << "ScenePanel paintEvent end";
}

void ScenePanel::dropEvent(QDropEvent *event)
{
}

void ScenePanel::dragEnterEvent(QDragEnterEvent *event)
{
}




void ScenePanel::slotAddItem(const QMimeData *mimeData,QPushButton* linkBtn)
{
    if(addArcPanel(mimeData,linkBtn)&&aliveObj<maxNumberBlock)
    {
       ++distributId;
        ++aliveObj;
    }   
    //qDebug()<<"mimeData->receive:"<<mimeData->text();
    this->update();
}

void ScenePanel::slotRemoveItem(const QMimeData *mimeData)
{

    if(removeArcPanel(mimeData)&&aliveObj>0)
    {
        --aliveObj;
    }
    //qDebug()<<"mimeData->receive:"<<mimeData->text();
    this->update();
}

void ScenePanel::slotExchangeItem(const QString objName, const QMimeData *mimeData)
{
    SubGphObj* tempSubGphObj=new SubGphObj;
    int leftObj=-1;
    int rightObj=-1;
    for(auto m_list_iter=linkGphObj_.begin();m_list_iter!=linkGphObj_.end();++m_list_iter)
    {
        if(m_list_iter.value()->getSubGphObjName()==objName)
        {

            leftObj=m_list_iter.key();
            qDebug()<<"leftObj : "<<leftObj;

        }
        if(m_list_iter.value()->getSubGphObjName()==mimeData->text())
        {
            rightObj=m_list_iter.key();
            qDebug()<<"rightObj : "<<rightObj;
        }
        if(leftObj!=-1&&rightObj!=-1)
            break;
    }
    if(leftObj!=-1&&rightObj!=-1)
    {
        tempSubGphObj=linkGphObj_[leftObj];
        linkGphObj_[leftObj]=linkGphObj_[rightObj];
        linkGphObj_[rightObj]=tempSubGphObj;
    }

    this->update();
}

//void ScenePanel::slotIsSelected(SubGphItem* item)
//{
//    for (auto m_list_iter = linkGphObj_.begin(); m_list_iter != linkGphObj_.end(); ++m_list_iter)
//    {
//        if (m_list_iter.value()->getGphItem() == item)
//        {
//            //m_list_iter.value()->set
//        }
//}
