#include "widget.h"
#include <QApplication>
#include <QString>
#include <QFileInfo>

int main(int argc, char *argv[])
{
//    QString filePath = argv[1]; //First arg is the running process
//    QFileInfo file(filePath);
//    if (file.exists(filePath))
//    {
//      QString name =file.fileName();
//      const QString fileName=file.absolutePath()+"\\"+ name + ".txt";
//      QFile myFile(fileName);
//      bool ok = myFile.open(QIODevice::WriteOnly);
//      if (ok)
//      {
//          const char *data=fileName.toStdString().c_str();
//          myFile.write(data);
//          myFile.close();
//      }
//        QString name =file.fileName();
//        const QString fileName=file.absolutePath()+"\\"+ name;
//     }


    QApplication a(argc, argv);

    Widget w;

    QString filePath =QString::fromLocal8Bit(argv[1]); //First arg is the running process
    QFileInfo file(filePath);
    if (file.exists(filePath))
    {
        QString name =file.fileName();
        const QString fileName=file.absolutePath()+"/"+ name;
        w.dragExeEvent(fileName);
    }

    w.show();

    return a.exec();
}
