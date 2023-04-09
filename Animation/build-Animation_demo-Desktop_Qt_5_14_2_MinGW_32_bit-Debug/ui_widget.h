/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget
{
public:
    QLabel *labelUnderline;
    QLabel *label_RightUp;
    QLabel *label_RightMid;
    QLabel *label_LeftUp;
    QLabel *label_LeftMid;
    QLabel *label_RightButtom;
    QLabel *label_LeftButtom;

    void setupUi(QWidget *Widget)
    {
        if (Widget->objectName().isEmpty())
            Widget->setObjectName(QString::fromUtf8("Widget"));
        Widget->resize(929, 720);
        labelUnderline = new QLabel(Widget);
        labelUnderline->setObjectName(QString::fromUtf8("labelUnderline"));
        labelUnderline->setGeometry(QRect(100, 350, 54, 12));
        label_RightUp = new QLabel(Widget);
        label_RightUp->setObjectName(QString::fromUtf8("label_RightUp"));
        label_RightUp->setGeometry(QRect(280, 0, 85, 86));
        label_RightMid = new QLabel(Widget);
        label_RightMid->setObjectName(QString::fromUtf8("label_RightMid"));
        label_RightMid->setGeometry(QRect(310, 70, 85, 177));
        label_LeftUp = new QLabel(Widget);
        label_LeftUp->setObjectName(QString::fromUtf8("label_LeftUp"));
        label_LeftUp->setGeometry(QRect(110, 50, 85, 86));
        label_LeftMid = new QLabel(Widget);
        label_LeftMid->setObjectName(QString::fromUtf8("label_LeftMid"));
        label_LeftMid->setGeometry(QRect(110, 120, 85, 177));
        label_RightButtom = new QLabel(Widget);
        label_RightButtom->setObjectName(QString::fromUtf8("label_RightButtom"));
        label_RightButtom->setGeometry(QRect(370, 190, 85, 177));
        label_LeftButtom = new QLabel(Widget);
        label_LeftButtom->setObjectName(QString::fromUtf8("label_LeftButtom"));
        label_LeftButtom->setGeometry(QRect(170, 240, 85, 177));

        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QCoreApplication::translate("Widget", "Widget", nullptr));
        labelUnderline->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
        label_RightUp->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
        label_RightMid->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
        label_LeftUp->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
        label_LeftMid->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
        label_RightButtom->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
        label_LeftButtom->setText(QCoreApplication::translate("Widget", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
