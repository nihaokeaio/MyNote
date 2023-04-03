#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QTextEdit>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QProcess>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    virtual ~Widget() override;

    void dragExeEvent(const QString fileName);

private slots:
    void on_selectFileBtn_clicked();

    void on_startBtn_clicked();

    void on_showFileText_textChanged();

    void getOrderPath(const QString& jsonPath);

    void createUpLinkJson(const QString& jsonPath);
    
    //bool eventFilter(QObject* obj, QEvent* e) override;
    //拖拽事件
    void dropEvent(QDropEvent *event) override;

    void dragEnterEvent(QDragEnterEvent *event) override;


private:
    Ui::Widget *ui;
private slots:
};

#endif // WIDGET_H
