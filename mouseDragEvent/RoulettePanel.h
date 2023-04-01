#ifndef ROULETTE_H
#define ROULETTE_H

#include <QWidget>
#include <QSet>
#include <QPushButton>
#include <QKeyEvent>

#include "ScenePanel.h"

class RoulettePanel : public QWidget
{
    Q_OBJECT
public:
    explicit RoulettePanel(QWidget* parent = nullptr);

    //���ÿɷ���İ�ť����
    void addLinkButton(QPushButton* button);

    //���ó�ʼԲ�̵Ĵ�С��outDiameter-�⾶��inDiameter-�ھ���gapSpacing-��϶
    void adjustRouletteSize(qreal outDiameter, qreal inDiameter,qreal gapSpacing);

    //���ó�ʼ�������̵İ�ť
    void setInitButtons(QVector<QPushButton*>buttons);

protected:
    bool eventFilter(QObject* o, QEvent* e);
    void performDrag(QWidget* widget);

    //�����¼�
    void keyPressEvent(QKeyEvent* event) override; //���̰����¼�
    void keyReleaseEvent(QKeyEvent* event) override; //�����ɿ��¼�

private:
    //\Բ�̶���ָ��
    ScenePanel* childPanel;

    //\���ڼ�����ק�¼��Ƿ���
    QPoint dragStartPostion_;

    //\���������Ҫ����������¼��İ�ť��������ק��ӵİ�ť
    QSet<QPushButton*>linkBtnSet_;

public slots:
    
};

#endif // WIDGET_H
