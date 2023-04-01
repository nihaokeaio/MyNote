#include "SubGphItem.h"
#include <QApplication>
#include <QGraphicsSceneEvent>
#include <QMouseEvent>
#include <QPen>
#include <QDebug>
#include <QDrag>
#include <QPushButton>


SubGphItem::SubGphItem(QGraphicsPathItem *parent) : QGraphicsPathItem(parent)
{
    gphtext=new QGraphicsSimpleTextItem(this);
    this->setAcceptDrops(true);
    isDeletePanel_=false;
    isExchangePanel_=true;
    isDraging_=false;
    linkBtn_=nullptr;
    btnFuncIsVaild_=false;
    this->setAcceptHoverEvents(true);
    //设置图形可被聚焦
    this->setFlag(QGraphicsItem::ItemIsFocusable,true);
    //\图形处于ACTIVE状态才能获得焦点
    this->setActive(true);
    this->setSelected(true);
    //this->setFlag(QGraphicsItem::ItemIsMovable);
    QRadialGradient gradient(0,0,100);
    gradient.setColorAt(0,Qt::white);
    gradient.setColorAt(1.0,qRgb(200,200,0));
    this->setBrush(gradient);
}

void SubGphItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
   if(event->type()==QEvent::GraphicsSceneMousePress)
   {
       if(event->button()==Qt::LeftButton)
       {
           dragStartPostion_=event->pos();
       }
    }
   //event->accept();
   //QGraphicsPathItem::mousePressEvent(event);

   
}

void SubGphItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    /*if(event->type()==QEvent::GraphicsSceneMouseMove)
    {
        if(event->type()==QMouseEvent::Enter)
        {
            QPen pen;
            pen.setColor(Qt::red);
            this->setPen(pen);
            qDebug()<<QString("subgphitem LeftButton Press");
        }
        if(event->type()==QMouseEvent::Leave)
        {
            QPen pen;
            pen.setColor(Qt::blue);
            this->setPen(pen);
            qDebug()<<QString("subgphitem RightButton Press");
        }
    }*/

    if (event->buttons() & Qt::LeftButton) {
        qreal distance = (event->pos() - dragStartPostion_).manhattanLength();
            if (distance >= QApplication::startDragDistance())
                performDrag();
        }
    
    QGraphicsPathItem::mouseMoveEvent(event);
}

void SubGphItem::performDrag()
{
    qDebug()<<this->objectName();
    QMimeData *mimeData = new QMimeData;
    QString strType=QString("type=%1-").arg(this->type());
    QString strPanel=QString("isDeletePanel=%1-").arg(this->isDeletePanel_);
    QString strObjName=QString("objectName=%1").arg(this->objectName());
    QString strInfo=strType+strPanel+strObjName;
    mimeData->setText(strInfo);
    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    isDraging_=true;
    //drag->setPixmap(QPixmap(":/images/res/QQ.png"));
    if (drag->exec(Qt::MoveAction) == Qt::MoveAction);

}

void SubGphItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if(event->type()==QEvent::GraphicsSceneMouseRelease)
    {
        if(event->button()==Qt::LeftButton)
        {
            //qDebug()<<QString("subgphitem LeftButton Release");        
            if(linkBtn_)
            {
                linkBtn_->click();
                //linkBtn_->isChecked();
            }
            else {
                qDebug()<<"subBtn_ is Null Value";
            }           
        }
    }
    
}

void SubGphItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    /*qDebug()<<"isPanel:"<<this->isPanel();
    qDebug()<<"isSelected:"<<this->isSelected();
    qDebug()<<"isWidget:"<<this->isWidget();
    qDebug()<<"isActive:"<<this->isActive();
    qDebug()<<"hasFocus:"<<this->hasFocus();
    qDebug()<<"isUnderMouse:"<<this->isUnderMouse();*/
    QGraphicsItem::hoverMoveEvent(event);
}

void SubGphItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    QRadialGradient gradient(0,0,100);
    gradient.setColorAt(0,Qt::white);
    gradient.setColorAt(1.0,qRgb(200,200,0));
    this->setBrush(gradient);
    btnFuncIsVaild_=false;
    this->clearFocus();
    //qDebug() << "SubGphItem::hoverLeaveEvent run";
    
    QGraphicsItem::hoverLeaveEvent(event);
}

void SubGphItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    //qDebug() << "SubGphItem::hoverEnterEvent run";
    btnFuncIsVaild_=true;
    QRadialGradient gradient(0,0,100);
    gradient.setColorAt(0,Qt::white);
    gradient.setColorAt(1.0,qRgb(100,100,0));
    this->setBrush(gradient);
    //\获得焦点
    this->setFocus();
    QGraphicsItem::hoverEnterEvent(event);
}

void SubGphItem::dropEvent(QGraphicsSceneDragDropEvent *event)
{
    QMimeData* mimeData=const_cast<QMimeData*>(event->mimeData());
    //\说明被拖拽的对象是grapItem，只有交换或者删除两种功能
    //qDebug()<<mimeData->text();
    if(mimeData->text().indexOf("type")==0)
    {
        QStringList list=mimeData->text().split('-');
        if(list.size()==3)
        {
            QStringList strObjName=list.back().split('=');
            mimeData->setText(strObjName[1]);
        }
        QStringList strPanel=list[1].split('=');
        int strisDeletePanel=1;
        if(strPanel.size()==2)
        {
            strisDeletePanel=strPanel[1].toInt();
        }

        if(this->isDeletePanel_==true&& strisDeletePanel==0 && isDraging_==false)
        {
            //qDebug()<<"Need delete widget"<<mimeData->text();
            emit removeGphItem(mimeData);
        }
        if(this->isDeletePanel_==false && strisDeletePanel==0 && isDraging_==false)
        {
            //qDebug()<<"Need Exchange between" << mimeData->text()<<" and "<< this->objectName();
            emit exchangeGphItem(this->objectName(),mimeData);
        }
    }
    //\此外，说明被拖拽对象为按钮，强转对象后，将其加入圆盘
    else if(this->isDeletePanel_ == false)
    {
        qDebug()<<event->source();
        QPushButton* linkBtn=qobject_cast<QPushButton*>(event->source());
        emit addGphItem(mimeData,linkBtn);
    }
    /*qDebug() << "SubGphItem focus:" << this->focusItem();
    this->setFocus();*/
    isDraging_=false;

}

void SubGphItem::dragEnterEvent(QGraphicsSceneDragDropEvent *event)
{
    event->acceptProposedAction();
    event->accept();
}

void SubGphItem::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_C)
    {
        qDebug() << "Key_C SubGphItem " << this->focusItem();
    }
    QGraphicsPathItem::keyPressEvent(event);
}

void SubGphItem::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Alt)
    {
        qDebug()<<"run button"<<this->objectName();
        if(btnFuncIsVaild_)
        {        
            if (linkBtn_)
            {
                linkBtn_->click();
                linkBtn_->isChecked();
            }
            else {
                qDebug() << "subBtn_ is Null Value";
            }           
        }
    }
    QGraphicsPathItem::keyReleaseEvent(event);
}

void SubGphItem::setTextPos(const QPointF &pos)
{
    textPos_=pos;
    gphtext->setPos(textPos_);
}

QPointF& SubGphItem::getTextPos()
{
    return textPos_;
}


bool SubGphItem::isDeletePanel()
{
    return isDeletePanel_;
}

void SubGphItem::setDeletePanel(bool flag)
{
    isDeletePanel_=flag;
}

bool SubGphItem::isExchangePanel()
{
    return  isExchangePanel_;
}

void SubGphItem::setExchangePanel(bool flag)
{
    isExchangePanel_=flag;
}

void SubGphItem::setLinkBtn(QPushButton *linkBtn)
{
    linkBtn_=linkBtn;
}

QPushButton *SubGphItem::getLinkBtn()
{
    return linkBtn_;
}





void SubGphItem::paintEvent(QPaintEvent *p)
{
    //setGradientArc(radius_,startAngle_,spanAngle_,arcHeight_,color_);
    qDebug() << "SubGphItem::paintEvent run";
    p->ignore();
    
}
