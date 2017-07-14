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

/*!
 * \brief Entidade responsável pelo desenvolvimento da janela de configuração do modelo e VANT
 *
 * A janela de configuração será onde o usuário visualizará dados de modelagem e
 * configurará o controlador com as sseguintes opções:
 * - nome do controlador
 * - tópicos dos sensores a serem utilizados
 * - tópicos dos atuadores a serem utilizados
 * - período de amostragem
 * - nome de arquivos para armazenamento de dados para uso em Matlab
 * - pose inicial
 */

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    void setModel(std::string,std::string); //! método que adquire os nomes e caminhos dos arquivos de descrção física e do controlador
    ~Dialog();

private slots:

    void on_pushButton_5_clicked(); // criando novo projeto de estratégia de controle

    void on_pushButton_7_clicked(); // compilando estratégia de controle

    void on_pushButton_6_clicked(); // abrindo projeto de controle já existente

    void on_pushButton_3_clicked(); // adicionando novo sensor

    void on_pushButton_4_clicked(); // removendo sensor

    void on_pushButton_clicked(); // adicionando controlador

    void on_pushButton_2_clicked(); // removendo controlador

    void on_buttonBox_accepted(); // botão de ok

    void SaveConfig(); // salvando dados no arquivo de configuração

protected:
    Ui::Dialog *ui;
private:
    Model model; // classe modelo
    Controller controller; // classe controlador
};

#endif // DIALOG_H
