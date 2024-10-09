#include "QtDemo.h"
#include <QIcon>
#include <QApplication>
#pragma comment(lib, "user32.lib")

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(":/logo.ico"));
    QtDemo w;
    w.show();
    return a.exec();
}