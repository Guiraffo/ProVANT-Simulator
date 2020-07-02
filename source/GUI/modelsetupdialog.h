#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "Business/model.h"
#include "Business/controller.h"
#include "dialognewcontroller.h"
#include "dialogopencontroller.h"

namespace Ui {
class ModelSetupDialog;
}

/*!
 * \brief Entidade responsável pelo desenvolvimento da janela de configuração
 * do modelo e VANT
 *
 * A janela de configuração será onde o usuário visualizará dados de modelagem e
 * configurará o controlador com as seguintes opções:
 * - nome do controlador
 * - tópicos dos sensores a serem utilizados
 * - tópicos dos atuadores a serem utilizados
 * - período de amostragem
 * - nome de arquivos para armazenamento de dados para uso em Matlab
 * - pose inicial
 */
class ModelSetupDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ModelSetupDialog(QWidget *parent = 0);
    //! método que adquire os nomes e caminhos dos arquivos de descrção física
    //! e do controlador
    void setModel(std::string,std::string);
    ~ModelSetupDialog();

    bool hil = false;

private slots:

    void on_newControllerButton_clicked(); // criando novo projeto de estratégia de controle

    void on_compileControllerButton_clicked(); // compilando estratégia de controle

    void on_openControllerButton_clicked(); // abrindo projeto de controle já existente

    void on_addSensorButton_clicked(); // adicionando novo sensor

    void on_removeSensorButton_clicked(); // removendo sensor

    void on_addActuatorButton_clicked(); // adicionando controlador

    void on_removeActuatorButton_clicked(); // removendo controlador

    void on_buttonBox_accepted(); // botão de ok

    void SaveConfig(); // salvando dados no arquivo de configuração

    void on_hilCheckBox_clicked(bool checked);


protected:
    Ui::ModelSetupDialog *ui;
private:
    Model model; // classe modelo
    Controller controller; // classe controlador
};

#endif // DIALOG_H
