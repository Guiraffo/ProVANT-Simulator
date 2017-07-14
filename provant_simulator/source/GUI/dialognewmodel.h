#ifndef DIALOGNEWMODEL_H
#define DIALOGNEWMODEL_H

#include <QDialog>
#include "mainwindow.h"

namespace Ui {
class Dialognewmodel;
}
/*!
 * \brief Entidade responsável por incluir novo modelo no cenário
 */
class Dialognewmodel : public QDialog
{
    Q_OBJECT

public:
    explicit Dialognewmodel(Ui::MainWindow*,QWidget *parent = 0);
    ~Dialognewmodel();
    void newModel(); //! método para adicionar todos os modelos existentes no ambiente de simulação no combox da tela de diálogo

private slots:
    void on_buttonBox_accepted(); // insere novo modelo na árvore de dados

private:
    void splitvector(std::string data,QTreeWidgetItem* Element); // divide uam string de pose em seis strings
    Ui::Dialognewmodel *ui;
    Ui::MainWindow* parentUi;
};

#endif // DIALOGNEWMODEL_H
