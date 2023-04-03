#include "MyFileDialog.h"


MyFileDialog::MyFileDialog(QWidget *parent):QFileDialog(parent)
{

}

void MyFileDialog::slotMyAccept()
{
    QDialog::accept();
}
