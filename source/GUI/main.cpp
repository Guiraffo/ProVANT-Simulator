#include "mainwindow.h"
#include <QApplication>
#include"DataAccess/GazeboElements/modelfile.h"
#include"DataAccess/GazeboElements/worldfile.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();

    return 0;
}
