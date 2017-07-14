#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "Business/model.h"
#include "Business/controller.h"
#include "dialognewcontroller.h"
#include "dialogopencontroller.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    void setModel(std::string,std::string);
    ~Dialog();

private slots:
    void on_comboBox_activated(const QString &arg1);

    void on_pushButton_5_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_6_clicked();

    void on_treeWidget_doubleClicked(const QModelIndex &index);

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_9_clicked();

    void on_buttonBox_accepted();

    void SaveConfig();

    void SaveParameters();

    void on_pushButton_8_clicked();

protected:
    Ui::Dialog *ui;
private:
    Model model;
    Controller controller;
};

#endif // DIALOG_H
