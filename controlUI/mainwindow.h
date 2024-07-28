#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public slots:
    void GoForward();
    void GoBack();
    void TurnRight();
    void TurnLeft();
    void TransmitParam(int);
    void TransmitParam(QString);
    void TransmitParam();
};

#endif // MAINWINDOW_H
