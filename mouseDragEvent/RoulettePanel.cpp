#include "RoulettePanel.h"
#include <QVBoxLayout>
#include <QMouseEvent>
#include <qapplication.h>
#include <QDrag>



RoulettePanel::RoulettePanel(QWidget* parent) : QWidget(parent)
{   
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setAttribute(Qt::WA_TranslucentBackground);
    this->resize(400, 600);

    QVBoxLayout* vlayout = new QVBoxLayout(this);  
    this->setTabletTracking(true);
    childPanel = new ScenePanel;
    vlayout->addWidget(childPanel);
    childPanel->setVisible(true);
}

void RoulettePanel::performDrag(QWidget* widget)
{
    QMimeData* mimeData = new QMimeData;
    mimeData->setText(widget->objectName());
    qDebug() << ("%f", mimeData);
    QDrag* drag = new QDrag(widget);
    drag->setMimeData(mimeData);
    //drag->setPixmap(QPixmap(":/images/person.png"));
    if (drag->exec(Qt::MoveAction) == Qt::MoveAction);

}

void RoulettePanel::addLinkButton(QPushButton* button)
{
    //button->acceptDrops();
    //button->setParent(this);
    button->installEventFilter(this);
    linkBtnSet_.insert(button);
}

void RoulettePanel::adjustRouletteSize(qreal outDiameter, qreal inDiameter, qreal gapSpacing)
{
    if (this->childPanel)
    {
        this->childPanel->adjustRouletteSize(outDiameter, inDiameter,gapSpacing);
    }
}

void RoulettePanel::setInitButtons(QVector<QPushButton*> buttons)
{
    for (auto button : buttons)
    {
        QMimeData* mimeData = new QMimeData;
        mimeData->setText(button->objectName());
        this->childPanel->slotAddItem(mimeData, button);
    }
    
}

bool RoulettePanel::eventFilter(QObject* o, QEvent* e)
{
    QPushButton* eventBtn = qobject_cast<QPushButton*>(o);
    if (linkBtnSet_.find(eventBtn) != linkBtnSet_.end())
    {
        if (e->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent* event = static_cast<QMouseEvent*>(e);
            if (event->button() == Qt::LeftButton)
            {
                dragStartPostion_ = event->pos();
                RoulettePanel::mousePressEvent(event);
            }
        }

        if (e->type() == QEvent::MouseMove)
        {
            QMouseEvent* event = static_cast<QMouseEvent*>(e);
            if (event->buttons() & Qt::LeftButton) {
                int distance = (event->pos() - dragStartPostion_).manhattanLength();
                if (distance >= QApplication::startDragDistance())
                {
                    performDrag(eventBtn);
                    eventBtn->clearFocus();
                }
            }
            RoulettePanel::mouseMoveEvent(event);
        }
    }

    return QWidget::eventFilter(o, e);
}

void RoulettePanel::keyPressEvent(QKeyEvent* event)
{
    /*if (event->key() == Qt::Key_Alt)
    {
        qDebug() << "Key_Alt keyPressEvent before " << this->focusWidget();
        childPanel->setVisible(true);
        childPanel->setFocus();
        qDebug() << "Key_Alt keyPressEvent after " << this->focusWidget();
    }

    if (event->key() == Qt::Key_C)
    {
        qDebug() << "Key_C keyPressEvent before " << this->focusWidget();
    }*/
    QWidget::keyPressEvent(event);
}

void RoulettePanel::keyReleaseEvent(QKeyEvent* event)
{
    /*if (event->key() == Qt::Key_Alt)
    {
        childPanel->setVisible(true);
    }*/
    QWidget::keyReleaseEvent(event);
}






