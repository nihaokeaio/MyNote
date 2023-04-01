#include "SubGphObj.h"

SubGphObj::SubGphObj(QObject *parent) : QObject(parent)
{
    isBuild_=false;
    objLabel_="Hay";
    subGphItem_=nullptr;
    subGphObjName_="Null_Value";
    subBtn_=nullptr;
}

void SubGphObj::setGphItem(SubGphItem *subpath)
{
    subGphItem_=subpath;
}

SubGphItem* SubGphObj::getGphItem()
{
    return subGphItem_;
}

bool SubGphObj::isbuild()
{
    return isBuild_;
}

void SubGphObj::setbuild(bool flag)
{
    isBuild_=flag;
}

//void SubGphObj::setLabelText(QString str)
//{
//    objLabel_=str;
//}

void SubGphObj::setSubGphItemText(QString str)
{
    if(subGphItem_)
    {
        subGphItem_->gphtext->setText(str);
    }
}

bool SubGphObj::isDeleteObj()
{
    if(subGphItem_)
    {
        return subGphItem_->isDeletePanel();
    }
    return false;
}

void SubGphObj::setDeleteObj(bool flag)
{
    if(subGphItem_)
    {
        subGphItem_->setDeletePanel(flag);
    }
}

void SubGphObj::setSubGphItemName(QString name)
{
    subGphObjName_=name;
    if(subGphItem_)
    {
        subGphItem_->setObjectName(subGphObjName_);
    }
}

QString SubGphObj::getSubGphObjName()
{
    return subGphObjName_;
}

bool SubGphObj::isExchangeObj()
{
    if(subGphItem_)
    {
        return subGphItem_->isExchangePanel();
    }
}

void SubGphObj::setExchangeObj(bool flag)
{
    if(subGphItem_)
    {
        subGphItem_->setExchangePanel(flag);
    }
}

//QString SubGphObj::getLabelText()
//{
//    qDebug()<<objLabel_;
//    return objLabel_;
//}

void SubGphObj::setLinkButton(QPushButton *linkBtn)
{
    subBtn_=linkBtn;
    qDebug() << subBtn_->parent();
}

QPushButton *SubGphObj::getLinkButton()
{
    return subBtn_;
}

//bool SubGphObj::eventFilter(QObject *obj, QEvent *event)
//{
//    if(obj==subGphItem_)
//    {
//        if(event->type()==QEvent::GraphicsSceneMouseRelease)
//        {
//            QMouseEvent* e=static_cast<QMouseEvent*>(event);
//            if(e->button()==Qt::LeftButton)
//            {
//                qDebug()<<QString("eventFilter subgphitem LeftButton Release");
//                if(this->objectName()=="NHKAO")
//                {
//                    if(subBtn_)
//                    {
//                        subBtn_->click();
//                        subBtn_->isChecked();
//                    }
//                    else {
//                        qDebug()<<"subBtn_ is Null Value";
//                    }
//                }
//                return true;
//            }
//            return false;
//        }
//
//    }
//    return QObject::eventFilter(obj,event);
//}





