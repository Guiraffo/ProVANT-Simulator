#ifndef DIALOGOPENCONTROLLER_H
#define DIALOGOPENCONTROLLER_H

#include <QDialog>
#include "qdebug.h"
#include "qfile.h"
#include "QDir"

namespace Ui {
class DialogOpenController;
}

/*!
 * \brief Entidade responsável por abrir o projeto de estratégia de controle desejado pelo usuário
 */
class DialogOpenController : public QDialog
{
    Q_OBJECT

public:
    explicit DialogOpenController(QWidget *parent = 0);
    ~DialogOpenController();
     void GetControllers(); //! adiciona todas estratégias existentes no ambiente de simulação no combobox

private slots:
    void on_buttonBox_accepted(); // abre pelo nautilus o projeto de estratégia de controle

private:
    Ui::DialogOpenController *ui;
};

#endif // DIALOGOPENCONTROLLER_H
