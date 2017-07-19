#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include "QApplication"
#include "QDesktopWidget"
#include "QFileDialog"
#include "qdebug.h"
#include "Business/world.h"
#include "Business/model.h"

namespace Ui {
class MainWindow;
}

/*!
 * \brief Entidade responsável pelo desenvolvimento da janela principal
 *
 * A janela principal será onde o usuário selecionará as configurações principais de simulação,definindo:
 * - passo de simulação
 * - motor de simulação
 * - cenário
 * - fator de tempo real
 * - aceleração da gravidade
 * - pose inicial
 */

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    /*
     *        Botão para implementação de rotina de inicilização do Gazebo com cenário, modelo e
     *        malha de controle corretamente configuradas
     */
    void on_pushButton_clicked();
    /*
     *        Opção para abertura de novo cenário de um template disponível pelo ambiente de simulação
     */
    void on_actionNew_triggered();

    /*
     *        Método que define a ação que será adotada quando se clica duas vezes em um dos itens da
     *        árvore de dados. Neste caso, apenas quando o item uri do modelo de vant for selecionado
     *        duas vezes, ele abrirá uma caixa de diálogo para visualizar e/ou editar configurações do
     *        modelo do VANT.
     */
    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);

    /*
     *        Método que define a ação que será adotada quando se clica uma vezes em um dos itens da árvore
     *        de dados. Neste caso, apenas quando o item uri do modelo de vant for selecionado uma vez, sua imagem
     *        poderá ser visualizada pelo usuário.
     */
    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

    /*
     *        Opção para abertura de cenário já existente.
     */
    void on_actionOpen_triggered();

    /*
     *        Método que encapsula lógica para salvar como dado cenário aberto na interface gráfica
     */
    bool SaveAs();

    /*
     *        Opção para salvar cenário aberto.
     */
    void on_actionSave_triggered();

    /*
     *        Opção para sair da interface gráfica.
     */
    void on_actionExit_triggered();

    /*
     *        Opção para incluir um novo modelo de vant.
     */
    void on_actionNew_2_triggered();

protected:
    Ui::MainWindow *ui;

private:
    world mundo; // classe para leitura e escrita de dados do cenário
    Model model; // classe para leitura e escrita de dados do cenário

    bool istemplate; // variável que armazena a informação sobre o cenário aberto é um template
};

#endif // MAINWINDOW_H
