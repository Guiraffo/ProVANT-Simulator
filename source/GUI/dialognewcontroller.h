#ifndef DIALOGNEWCONTROLLER_H
#define DIALOGNEWCONTROLLER_H

#include <QDialog>

namespace Ui {
class DialogNewController;
}

/*!
 * \brief The DialogNewController class shows a dialog to allow the user to
 * input the name of a new controller to be created.
 */
class DialogNewController : public QDialog
{
    Q_OBJECT

public:
    explicit DialogNewController(QWidget *parent = 0);
    ~DialogNewController();

private:
    Ui::DialogNewController *ui;

public slots:
    void accept();
};

#endif // DIALOGNEWCONTROLLER_H
