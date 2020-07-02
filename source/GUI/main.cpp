#include <QApplication>

#include "mainwindow.h"

/**
 * @brief main Entry point for the simulator GUI.
 * @param argc Number of arguments passed to the program.
 * @param argv String vector containted the arguments passed to the program.
 * @return Integer code with program execution status.
 */
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
