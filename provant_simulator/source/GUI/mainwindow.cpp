
#include "mainwindow.h"
#include "dialog.h"
#include "dialognewmodel.h"
#include "DataAccess/RosElements/roslaunch.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->treeWidget->setColumnCount(2); // arvore de dados terá duas colunas
    // inicialmente não se pode editar árvore de dados
    ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);
    // quando a interface é aberta, não há cenário selecionado
    // então não é permitido adicionar modelo, salvar cenário e nem
    // inicioalizar uma simulação
    ui->actionSave->setDisabled(true);
    ui->menuEdit->setDisabled(true);
    ui->pushButton->setDisabled(true);
    // nome que será observado na parte superior da janela
    const QString name("Provant Simulator");
    setWindowTitle(name);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    for(int i=0;i<ui->treeWidget->topLevelItemCount();i++)
    {
        // analisar todos os os modelos do gazebo existentes no mundo
        if(ui->treeWidget->topLevelItem(i)->text(0)=="Include")
        {
            // adquire a URL onde estão armazenados todos os modelos de vant do ambiente de simulação
                char const* tmp = getenv( "GAZEBO_MODEL_PATH" );
                if ( tmp == NULL ) {
                    qDebug() << "Problemas com variavel de ambiente ";
                } else {
                    std::string env(tmp);
                    QDir dir(env.c_str());
                    QFileInfoList files = dir.entryInfoList();
            // -------------------------------------------------------------------------------------

                foreach (QFileInfo file, files)
                {
                    if (file.isDir())
                    {
                        // determinou-se que o terceiro include, corresponde sempre ao modelo do vant
                        if(file.fileName() == ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://",""))
                        {
                            QString qenv(env.c_str());
                            qenv = qenv + "/" +  ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://","");
                            qenv = qenv + "/config/config.xml";
                            qDebug() << qenv;
                            // informa o arquivo de configuração da malha de controle configurada na interface de simulação
                            // para dado vant (arquivo se ecnontra dentro da pasta config com o nome config.xml)
                            std::string command("export TILT_CONFIG="+qenv.toStdString());
                            std::system(command.c_str());
                        }
                    }
                }
            }
        }
    }

    if(istemplate) // caso seja template, o usuário é obrigado a adicionar informações em um arquivo .world
    {
        if(SaveAs())
        {
            roslaunch::WriteNew(QString::fromStdString(mundo.actualword->Filename)); // configurando rotina de inicilização do simulador
            std::system("roslaunch simulation_elements gazebo.launch"); // comando de inicializar simulação
        }
    }
    else
    {
        mundo.Write(ui->treeWidget); // salvar modificações no mesmo arquivo de destino
        roslaunch::WriteNew(QString::fromStdString(mundo.actualword->Filename)); // configurando rotina de inicilização do simulador
        std::system("roslaunch simulation_elements gazebo.launch"); // comando de inicializar simulação
    }
}

void MainWindow::on_actionNew_triggered()
{
    // Abertura de caixa de diálogo para selecionar arquivo template
    QString filename = QFileDialog::getOpenFileName(this
                                                    ,tr("New World")
                                                    , "/home"
                                                    , tr("Template Files (*.tpl)"));
    if(filename.isEmpty()) return;

    // filtrando string obtida
    QString dir;
    QStringList splitvector;
    QRegExp rx("\\/");
    splitvector = filename.split(rx);
    splitvector.removeLast();
    foreach (const QString &str, splitvector)
    {
        if (str.contains(" ")||str.size()==0)
        {
            //faz nada
        }
        else
        {
            dir = dir+"/"+str;
        }
    }

    // colocando imagem do mundo fornecida pelo usuário  na interface gráfica
    QString imagefile(dir+"/imagem.gif");
    QFile ff(imagefile);
    QFileInfo fileInfo(ff);
    if (fileInfo.exists())
    {
        QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView_2);
        QImage image(imagefile);
        scene->addPixmap(QPixmap::fromImage(image));
        ui->graphicsView_2->setScene(scene);
        ui->graphicsView_2->show();
    }

    ui->treeWidget->clear();
    // lê dados do arquivo .tpl informado anteriormnete
    mundo.getFirst(filename.toStdString(),ui->treeWidget);

    // habilitando funções de menuu e inicialização da interface
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->pushButton->setEnabled(true);

    // informa que o arquivo ainda é template, então usuário
    // deve salvar os dados em outro arquivo com formato .world
    istemplate = true;
}


void MainWindow::on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column)
{
    // permite edição de dados na segunda coluna da árvore de dados
    if (column == 1 && item->text(0) != "uri") {
            ui->treeWidget->editItem(item, column);
        }

    // se campo for do tipo uri, abrir diálogo de configuração do modelo
    if(item->text(0) == "uri")
    {
        char const* tmp = getenv("GAZEBO_MODEL_PATH");
        if ( tmp == NULL ) {
            qDebug() << "Problemas com variavel de ambiente ";
        } else {
            // obter local com a url do modelo
            std::string env(tmp);
            QDir dir(env.c_str());
            QStringList list;
            list = item->text(1).split("//");
            if (!dir.cd(QString::fromStdString(list.at(1).toStdString()))) // "/tmp"
            {
                // não faz nada
            }
            else
            {
                // arquivo de descrição física
                std::string localmodel = env+"/"+list.at(1).toStdString()+"/robot/model.sdf";
                // arquivo de configuração de controladoor
                std::string localconfig = env+"/"+list.at(1).toStdString()+"/config/config.xml";
                // abrir janela
                Dialog modelsetup;
                modelsetup.setModel(localmodel,localconfig);
                modelsetup.setModal(true);
                modelsetup.exec();
            }
        }
    }
}


void MainWindow::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{
    if(item->text(0) == "uri")
    {
        char const* tmp = getenv( "GAZEBO_MODEL_PATH" );
        if ( tmp == NULL ) {
            qDebug() << "Problemas com variavel de ambiente ";
        } else {
            std::string env(tmp);
            QDir dir(env.c_str());
            QStringList list;
            list = item->text(1).split("//");
            // não há arquivo
            if (!dir.cd(QString::fromStdString(list.at(1).toStdString()))) // "/tmp"
            {
                QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView);
                ui->graphicsView->setScene(scene);
                ui->graphicsView->show();
            }
            else // há arquivo
            {
                // plotando foto do vant
                QString imagefile(QString::fromStdString(env)+"/"+list.at(1)+"/imagem.gif");
                QFile ff(imagefile);
                QFileInfo fileInfo(ff);
                if (fileInfo.exists())
                {
                    QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView);
                    QImage image(imagefile);
                    scene->addPixmap(QPixmap::fromImage(image));
                    ui->graphicsView->setScene(scene);
                    ui->graphicsView->show();
                }
            }
        }
    }
}

void MainWindow::on_actionOpen_triggered()
{
    // escolhe do arquivo .world
    QString filename = QFileDialog::getOpenFileName(this
                                                    ,tr("Open World")
                                                    , "/home/macro/catkin_ws/src/provant_simulator/source/Database"
                                                    , tr("World Files (*.world)"));
    if(filename.isEmpty()) return;
    // tratando nome do arquivo
    QString dir;
    QStringList splitvector;
    QRegExp rx("\\/");
    splitvector = filename.split(rx);
    splitvector.removeLast();
    // filter
    foreach (const QString &str, splitvector)
    {
        if (str.contains(" ")||str.size()==0)
        {
            //faz nada
        }
        else
        {
            dir = dir+"/"+str;
        }
    }

    // iamgem do cenário
    QString imagefile(dir+"/imagem.gif");
    QFile ff(imagefile);
    QFileInfo fileInfo(ff);
    if (fileInfo.exists())
    {
        QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView_2);
        QImage image(imagefile);
        scene->addPixmap(QPixmap::fromImage(image));
        ui->graphicsView_2->setScene(scene);
        ui->graphicsView_2->show();
    }
    // limpando árvore para adicionar novos dados
    ui->treeWidget->clear();
    // adicionando dados na árvore de dados
    mundo.getFirst(filename.toStdString(),ui->treeWidget);
    // habilitando funções de menuu e inicialização da interface
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->pushButton->setEnabled(true);
    istemplate = false; // não é template
}


bool MainWindow::SaveAs()
{
    QString sf;
    // escolher nome do arquivo e diretório
    QString filename = QFileDialog::getSaveFileName(this,
                                 tr("Save Word"),
                                 "/home/macro/catkin_ws/src/provant_simulator/source/Database",
                                 tr("World Files (*.world)"),
                                 &sf);
    if(filename.isEmpty()) return false;

    // geraciamneto de sufixo
    QFileInfo f( filename );
    if (f.suffix().isEmpty())
    {
        // http://www.qtcentre.org/forum/f-qt-programming-2/t-qfiledialoggetsavefilename-default-extension-8503.html
        filename += ".world";
    }
    else
    {
        if(f.suffix()!=".world")filename += ".world";
    }

    mundo.actualword->Filename = filename.toStdString();
    // atualizar arquivo
    mundo.Write(ui->treeWidget);
    // não é template
    istemplate = false;
    return true;
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::on_actionNew_2_triggered()
{
    Dialognewmodel newmodel(ui,this);
    newmodel.newModel();
    newmodel.setModal(true);
    newmodel.exec();
}

void MainWindow::on_actionSave_triggered()
{
    SaveAs();
}
