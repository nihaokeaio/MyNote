#ifndef MYFILEDIALOG_H
#define MYFILEDIALOG_H
#include <QFileDialog>
#include <QWidget>

class MyFileDialog : public QFileDialog
{
    Q_OBJECT
public:
    explicit MyFileDialog(QWidget* parent=nullptr);

public slots:
    void slotMyAccept();

};

#endif // MYFILEDIALOG_H
