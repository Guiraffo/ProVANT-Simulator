#ifndef DIALOGOPENCONTROLLER_H
#define DIALOGOPENCONTROLLER_H

#include <QDialog>
#include "qdebug.h"
#include "qfile.h"
#include "QDir"

namespace Ui {
class DialogOpenController;
}

class DialogOpenController : public QDialog
{
    Q_OBJECT

public:
    explicit DialogOpenController(QWidget *parent = 0);
    ~DialogOpenController();
    void GetControllers();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogOpenController *ui;
};

#endif // DIALOGOPENCONTROLLER_H
