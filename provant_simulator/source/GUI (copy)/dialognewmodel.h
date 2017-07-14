#ifndef DIALOGNEWMODEL_H
#define DIALOGNEWMODEL_H

#include <QDialog>
#include "mainwindow.h"

namespace Ui {
class Dialognewmodel;
}

class Dialognewmodel : public QDialog
{
    Q_OBJECT

public:
    explicit Dialognewmodel(Ui::MainWindow*,QWidget *parent = 0);
    ~Dialognewmodel();
    void newModel();
    void splitvector(std::string data,QTreeWidgetItem* Element);

private slots:
    void on_buttonBox_accepted();

private:
    Ui::Dialognewmodel *ui;
    Ui::MainWindow* parentUi;
};

#endif // DIALOGNEWMODEL_H
