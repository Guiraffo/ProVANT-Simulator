#ifndef JOINTSDIALOG_H
#define JOINTSDIALOG_H

#include <QDialog>
#include "QTreeWidgetItem"

namespace Ui {
class JointsDialog;
}

class JointsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit JointsDialog(QWidget *parent = 0);
    ~JointsDialog();

private slots:
    void on_buttonBox_accepted();

    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);

private:
    Ui::JointsDialog *ui;
};

#endif // JOINTSDIALOG_H
