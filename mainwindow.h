#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "CGAL_defines.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    QString filename;
    Arrangement_2 arr;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private Q_SLOTS:
   void on_pushButton_3_clicked();
   void on_actionOpen_Obstacle_File_triggered();

   void on_actionOpen_Robot_File_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
