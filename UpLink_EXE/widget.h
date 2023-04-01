#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QTextEdit>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_selectFileBtn_clicked();

    void on_startBtn_clicked();

    void getOrderPath(const QString& jsonPath);

    void createUpLinkJson(const QString& jsonPath);
    
    bool eventFilter(QObject* obj, QEvent* e) override;
private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
