#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->sliderKP->setRange(0, 99);

    localDevice = new QBluetoothLocalDevice();
//    qDebug() << (localDevice->isValid());
//    discoveryAgent= new QBluetoothDeviceDiscoveryAgent();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::GoForward()
{

}

void MainWindow::GoBack()
{

}

void MainWindow::TurnRight()
{

}
void MainWindow::TurnLeft()
{

}

void MainWindow::TransmitParam(int)
{
    ui->lineKP->setText(QString("%1").arg(ui->sliderKP->value()).toLatin1());
    ui->lineKI->setText(QString("%1").arg(ui->sliderKI->value()).toLatin1());
    ui->lineKD->setText(QString("%1").arg(ui->sliderKD->value()).toLatin1());
}

void MainWindow::TransmitParam(QString)
{

}

void MainWindow::TransmitParam()
{
    ui->sliderKP->setValue(ui->lineKP->text().toInt());
    ui->sliderKI->setValue(ui->lineKI->text().toInt());
    ui->sliderKD->setValue(ui->lineKD->text().toInt());
}
