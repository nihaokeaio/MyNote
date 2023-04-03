#include "widget.h"
#include "ui_widget.h"
#include "MyFileDialog.h"
#include <QHBoxLayout>
#include <QFileDialog>
#include <QListView>
#include <QTreeView>
#include <QDialogButtonBox>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QProcess>
#include <QSettings>
#include <QTime>
#include <QDropEvent>
#include <QMimeData>
#include <QFile>


Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setObjectName("UP3DLinkOrder");
    ui->selectFileBtn->setText("拖入订单文件夹或PathLocator.json文件");
    ui->startBtn->setText("启动订单");

    this->setAcceptDrops(true);
    ui->showFileText->setAcceptDrops(false);
    ui->showFileText->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    //ui->showStatusLabel->setVisible(true);
    ui->showStatusLabel->setText("");
    this->setWindowTitle("UP3DLinkOrderStarter.exe");
    this->setWindowFlags(this->windowFlags()&~Qt::WindowMinMaxButtonsHint|Qt::WindowMinimizeButtonHint);
    this->setFixedSize(700,120);

    //这是在Qt的资源下的文件,可以不用在资源下
    QFile file(":/qss/style.qss");
    //只读方式打开文件
    file.open(QFile::ReadOnly);
    //读取文件的所有内容，并转换成QString类型
    QString styleSheet = tr(file.readAll());
    //当前窗口设置样式表
    this->setStyleSheet(styleSheet);
    qDebug()<<"this objectname:"<<this->objectName();



}


Widget::~Widget()
{
    delete ui;
}

void Widget::on_selectFileBtn_clicked()
{
    MyFileDialog* dialag=new MyFileDialog(this);
    dialag->setOption(QFileDialog::DontUseNativeDialog,true);

    QListView* listView=dialag->findChild<QListView*>("listView");
    if(listView)
    {
        listView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    }
    QTreeView* treeView=dialag->findChild<QTreeView*>();
    if(treeView)
    {
        treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    }
    QDialogButtonBox* button=dialag->findChild<QDialogButtonBox*>("buttonBox");

    disconnect(button,SIGNAL(accepted()),dialag,SLOT(accept()));
    connect(button,SIGNAL(accepted()),dialag,SLOT(slotMyAccept()));
    if(dialag->exec()==QDialog::Accepted)
    {
        QStringList list=dialag->selectedFiles();
        if(list.size()!=1)
        {
            ui->showStatusLabel->setText("请选择一个订单");
            return;
        }
        ui->showFileText->setText(list[0]);
    }


}

void Widget::on_startBtn_clicked()
{
    if(ui->showFileText->toPlainText()!="")
    {
        QString filePath=ui->showFileText->toPlainText();
        QFileInfo fileInfo(filePath);
        if(fileInfo.isDir())
        {
            filePath+="/PathLocator.json";
        }
        fileInfo=QFileInfo(filePath);
        if(fileInfo.exists()&&fileInfo.suffix()=="json")
        {
            qDebug()<<filePath;
            qDebug()<<"Open success";
            getOrderPath(filePath);
            createUpLinkJson(filePath);


            QSettings reg("HKEY_LOCAL_MACHINE\\SOFTWARE\\UP3D\\UPCAD", QSettings::NativeFormat);
            QString strCAD = reg.value("Path").toString();

            QProcess* process = new QProcess(this);
            /*QString workpath = "E:/Bin/";
            process->setWorkingDirectory(workpath);*/
                  
            QString path = "\"" + strCAD + "UP3D.Designer.exe" + "\"";
            path=QDir::fromNativeSeparators(path);

            QStringList paramList;

            QDateTime current_date_time = QDateTime::currentDateTime();
            QString current_date = current_date_time.toString("ss");

            paramList << QString("ordersid=%1").arg(current_date);
            paramList << "orderSource = UPLink";
            paramList << "uplinkLanguage=chinese";

            //process->startDetached("E:/Bin/QQ.exe",paramList);
            ui->showStatusLabel->setText("订单启动中，请稍后...");
            if(process->startDetached(path, paramList))
            {
                ui->showStatusLabel->setText("订单启动成功");
            }
            else
            {
                ui->showStatusLabel->setText("订单启动失败");
            }
        }
        else
        {
            ui->showStatusLabel->setText("未找到订单相应的PathLocator.json文件");
        }
    }
}


void Widget::getOrderPath(const QString& jsonPath)
{
  
    QFileInfo fileInfo(jsonPath);

    QString absoluteDir = fileInfo.absoluteDir().path();

    QJsonObject newJsonObj;
    QString inputPath = QDir::toNativeSeparators(absoluteDir) + "\\UP_SourceFile_v2\\";
    newJsonObj.insert(QString("inputPath"), inputPath);

    QString outputPath = QDir::toNativeSeparators(absoluteDir) + "\\UP_OutputFile\\";
    newJsonObj.insert(QString("outputPath"), outputPath);
    
    QString cadTempPath = QDir::toNativeSeparators(absoluteDir) + "\\UP_Temp\\";
    newJsonObj.insert(QString("cadTempPath"), cadTempPath);
    

    QJsonDocument jsonDoc(newJsonObj);
    QByteArray data = jsonDoc.toJson();
    QFile file(QDir::toNativeSeparators(absoluteDir) + "\\PathLocator.json");
    bool ok = file.open(QIODevice::WriteOnly);
    if (ok)
    {
        file.write(data);
        file.close();
    }
    else
    {
        qDebug() << "write error";
        ui->showStatusLabel->setText("写入订单文件PathLocator.json失败");
    }

}

void Widget::createUpLinkJson(const QString& jsonPath)
{
    QFileInfo fileInfo(jsonPath);

    QString absoluteDir = fileInfo.absoluteDir().path();

    QJsonObject newJsonObj;
    QString inputPath = QDir::toNativeSeparators(absoluteDir);

    newJsonObj.insert(QString("designPath"), inputPath);
    newJsonObj.insert(QString("cadType"), "1");
    newJsonObj.insert(QString("cadFilePath"), "");
    newJsonObj.insert(QString("cadVersion"), "");


    QJsonDocument jsonDoc(newJsonObj);
    QByteArray data = jsonDoc.toJson();
    const QString fileName=absoluteDir +"\\UPLink.json";
    QFile file(fileName);
    bool ok = file.open(QIODevice::WriteOnly);
    if (ok)
    {
        file.write(data);
        file.close();
    }
    else
    {
        qDebug() << "write error";
        ui->showStatusLabel->setText("创建订单文件UPLink.json失败");
    }

}

void Widget::dropEvent(QDropEvent *event)
{
    QList<QUrl>urlList = event->mimeData()->urls();
    if (urlList.size()!=1) {
        return;
    }
    ui->showFileText->clear();
    QString textPath = urlList.first().toLocalFile();
    qDebug() << "原始路径:" << urlList.first();
    qDebug() << "处理后路径:" << textPath;
    ui->showFileText->setText(textPath);
}

void Widget::dragEnterEvent(QDragEnterEvent *event)
{
    if(event->mimeData()->hasUrls())
    {
        event->acceptProposedAction();
    }
    else
    {
        event->ignore();
    }
}


void Widget::on_showFileText_textChanged()
{
    ui->showStatusLabel->clear();
}

void Widget::dragExeEvent(const QString fileName)
{
    ui->showFileText->setText(fileName);
    on_startBtn_clicked();
}

