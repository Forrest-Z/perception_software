#include "modules/perception/camera/tools/qt_show/widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    apollo::cyber::Init(argv[0]);
    QApplication a(argc, argv);
    Widget w;
    w.show();

    return a.exec();
}
