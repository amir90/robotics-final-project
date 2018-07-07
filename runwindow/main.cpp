#include "runwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RunWindow w;
    w.show();

    return a.exec();
}
