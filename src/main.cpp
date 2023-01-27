#include "control_laser.h"

#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    control_laser w;
    w.show();
    return a.exec();
}
