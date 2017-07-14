#ifndef DIALOGNFIELDS_H
#define DIALOGNFIELDS_H

#include <QDialog>
#include "dialog.h"

namespace Ui {
class DialogNfields;
}

class DialogNfields : public QDialog
{
    Q_OBJECT

public:
    explicit DialogNfields(QWidget *parent = 0);
    ~DialogNfields();
    int Nfields();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogNfields *ui;
    int n;
};

#endif // DIALOGNFIELDS_H
