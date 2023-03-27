#include "panelobj.h"

SubPanelObj::SubPanelObj(QObject *parent) : QObject(parent)
{
    isbuild_=false;
    objLabel_="Hay";
    subGphItem_=nullptr;
    PanelObjName_="Null_Value";
    subBtn_=nullptr;
}

void SubPanelObj::setGphItem(SubGphItem *subpath)
{
    subGphItem_=subpath;
}

SubGphItem* SubPanelObj::getGphItem()
{
    return subGphItem_;
}

bool SubPanelObj::isbuild()
{
    return isbuild_;
}

void SubPanelObj::setbuild(bool flag)
{
    isbuild_=flag;
}

void SubPanelObj::setLabelText(QString str)
{
    objLabel_=str;
}

void SubPanelObj::setSubGphItemText(QString str)
{
    if(subGphItem_)
    {
        subGphItem_->gphtext->setText(str);
    }
}

bool SubPanelObj::isDeleteObj()
{
    if(subGphItem_)
    {
        return subGphItem_->isDeletePanel();
    }
    return false;
}

void SubPanelObj::setDeleteObj(bool flag)
{
    if(subGphItem_)
    {
        subGphItem_->setDeletePanel(flag);
    }
}

void SubPanelObj::setPanelObjName(QString name)
{
    PanelObjName_=name;
    if(subGphItem_)
    {
        subGphItem_->setObjectName(PanelObjName_);
    }
}

QString SubPanelObj::getPanelObjName()
{

    return PanelObjName_;
}

bool SubPanelObj::isExchangeObj()
{
    if(subGphItem_)
    {
        return subGphItem_->isExchangePanel();
    }
}

void SubPanelObj::setExchangeObj(bool flag)
{
    if(subGphItem_)
    {
        subGphItem_->setExchangePanel(flag);
    }
}

QString SubPanelObj::getLabelText()
{
    qDebug()<<objLabel_;
    return objLabel_;
}

void SubPanelObj::manualInstallFilter()
{
    if(subGphItem_)
    {
        subGphItem_->installEventFilter(this);
    }
}


void SubPanelObj::setLinkButton(QPushButton *linkBtn)
{
    subBtn_=linkBtn;
}

QPushButton *SubPanelObj::getLinkButton()
{
    return subBtn_;
}

bool SubPanelObj::eventFilter(QObject *obj, QEvent *event)
{
    if(obj==subGphItem_)
    {
        if(event->type()==QEvent::GraphicsSceneMouseRelease)
        {
            QMouseEvent* e=static_cast<QMouseEvent*>(event);
            if(e->button()==Qt::LeftButton)
            {
                qDebug()<<QString("eventFilter subgphitem LeftButton Release");
                if(this->objectName()=="NHKAO")
                {
                    if(subBtn_)
                    {
                        subBtn_->click();
                        subBtn_->isChecked();
                    }
                    else {
                        qDebug()<<"subBtn_ is Null Value";
                    }
                }
                return true;
            }
            return false;
        }

    }
    return QObject::eventFilter(obj,event);
}





