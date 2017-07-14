#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include "QApplication"
#include "QDesktopWidget"
#include "QFileDialog"
#include "qdebug.h"
#include "Business/world.h"
#include "Business/model.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_actionNew_triggered();

    void on_treeWidget_doubleClicked(const QModelIndex &index);

    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);

    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

    void on_actionOpen_triggered();

    bool SaveAs();

    void Save();

    void on_actionSave_triggered();

    void on_actionExit_triggered();

    void on_actionEdit_triggered();

    void on_actionNew_2_triggered();

protected:
    Ui::MainWindow *ui;

private:
    world mundo;
    Model model;

    bool istemplate;
};

#endif // MAINWINDOW_H
