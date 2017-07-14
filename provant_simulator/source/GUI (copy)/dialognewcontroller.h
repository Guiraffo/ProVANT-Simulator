#ifndef DIALOGNEWCONTROLLER_H
#define DIALOGNEWCONTROLLER_H

#include <QDialog>
#include "DataAccess/ControllerElements/newstrategy.h"

namespace Ui {
class DialogNewController;
}

class DialogNewController : public QDialog
{
    Q_OBJECT

public:
    explicit DialogNewController(QWidget *parent = 0);
    ~DialogNewController();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogNewController *ui;
};

#endif // DIALOGNEWCONTROLLER_H
