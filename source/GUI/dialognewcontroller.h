#ifndef DIALOGNEWCONTROLLER_H
#define DIALOGNEWCONTROLLER_H

#include <QDialog>
#include "DataAccess/ControllerElements/newstrategy.h"

namespace Ui {
class DialogNewController;
}

/*!
 * \brief Entidade responsável pelo desenvolvimento da janela para escolha do novo do novo projeto de controle a ser desenvolvido
 */
class DialogNewController : public QDialog
{
    Q_OBJECT

public:
    explicit DialogNewController(QWidget *parent = 0);
    ~DialogNewController();

private slots:
    void on_buttonBox_accepted(); // botão ok

private:
    Ui::DialogNewController *ui;
};

#endif // DIALOGNEWCONTROLLER_H
