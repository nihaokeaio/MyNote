/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget
{
public:
    QWidget *verticalLayoutWidget;
    QVBoxLayout *globalLayout;
    QHBoxLayout *fileHLay;
    QPushButton *selectFileBtn;
    QTextEdit *showFileText;
    QSpacerItem *vSpacer;
    QHBoxLayout *hLayoutDown;
    QLabel *showStatusLabel;
    QSpacerItem *hSpacer;
    QPushButton *startBtn;

    void setupUi(QWidget *Widget)
    {
        if (Widget->objectName().isEmpty())
            Widget->setObjectName(QString::fromUtf8("Widget"));
        Widget->resize(550, 115);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Widget->sizePolicy().hasHeightForWidth());
        Widget->setSizePolicy(sizePolicy);
        verticalLayoutWidget = new QWidget(Widget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(20, 10, 511, 91));
        globalLayout = new QVBoxLayout(verticalLayoutWidget);
        globalLayout->setSpacing(6);
        globalLayout->setContentsMargins(11, 11, 11, 11);
        globalLayout->setObjectName(QString::fromUtf8("globalLayout"));
        globalLayout->setContentsMargins(0, 0, 0, 0);
        fileHLay = new QHBoxLayout();
        fileHLay->setSpacing(6);
        fileHLay->setObjectName(QString::fromUtf8("fileHLay"));
        selectFileBtn = new QPushButton(verticalLayoutWidget);
        selectFileBtn->setObjectName(QString::fromUtf8("selectFileBtn"));

        fileHLay->addWidget(selectFileBtn);

        showFileText = new QTextEdit(verticalLayoutWidget);
        showFileText->setObjectName(QString::fromUtf8("showFileText"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(showFileText->sizePolicy().hasHeightForWidth());
        showFileText->setSizePolicy(sizePolicy1);
        showFileText->setMaximumSize(QSize(16777215, 20));
        showFileText->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        showFileText->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        showFileText->setLineWrapMode(QTextEdit::NoWrap);

        fileHLay->addWidget(showFileText);


        globalLayout->addLayout(fileHLay);

        vSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        globalLayout->addItem(vSpacer);

        hLayoutDown = new QHBoxLayout();
        hLayoutDown->setSpacing(6);
        hLayoutDown->setObjectName(QString::fromUtf8("hLayoutDown"));
        showStatusLabel = new QLabel(verticalLayoutWidget);
        showStatusLabel->setObjectName(QString::fromUtf8("showStatusLabel"));

        hLayoutDown->addWidget(showStatusLabel);

        hSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hLayoutDown->addItem(hSpacer);

        startBtn = new QPushButton(verticalLayoutWidget);
        startBtn->setObjectName(QString::fromUtf8("startBtn"));

        hLayoutDown->addWidget(startBtn);


        globalLayout->addLayout(hLayoutDown);


        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QApplication::translate("Widget", "Widget", 0, QApplication::UnicodeUTF8));
        selectFileBtn->setText(QApplication::translate("Widget", "PushButton", 0, QApplication::UnicodeUTF8));
        showStatusLabel->setText(QApplication::translate("Widget", "statusInfo", 0, QApplication::UnicodeUTF8));
        startBtn->setText(QApplication::translate("Widget", "PushButton", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
