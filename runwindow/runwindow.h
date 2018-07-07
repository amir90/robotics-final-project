#ifndef RUNWINDOW_H
#define RUNWINDOW_H

#include <QMainWindow>

namespace Ui {
class RunWindow;
}

class RunWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit RunWindow(QWidget *parent = 0);
    ~RunWindow();

private:
    Ui::RunWindow *ui;
};

#endif // RUNWINDOW_H
