/**
  * @file mainwindow.cpp
  * @author Arthur Viana Lara
  * @date 18/05/2018
  * Projeto: ProVANT
  */

#include "mainwindow.h"

#include <fcntl.h>

#include "Utils/appsettings.h"

#include "applicationsettingsdialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Inserindo Logo
    QPixmap provantLogo(":/logos/logos/provant_ufmg_ufsc_fundoalpha.png");
    int w = ui->provantLogoDisplay->width();
    int h = ui->provantLogoDisplay->height();
    ui->provantLogoDisplay->setPixmap(
                provantLogo.scaled(w,h,Qt::KeepAspectRatio)
    );

    // Configurando árvore de dados
    // arvore de dados terá duas colunas
    ui->treeWidget->setColumnCount(2);
    // inicialmente não se pode editar árvore de dados
    ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);

    // quando a interface é aberta, não há cenário selecionado
    // então não é permitido adicionar modelo, salvar cenário e nem
    // inicioalizar uma simulação
    ui->actionSave->setDisabled(true);
    ui->menuEdit->setDisabled(true);
    ui->startGazeboPushButton->setDisabled(true);
    ui->jointValuesPushButton->setDisabled(true);

    // nome que será observado na parte superior da janela
    const QString name("ProVANT Simulator v1");
    setWindowTitle(name);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_startGazeboPushButton_clicked()
{
    for(int i=0;i<ui->treeWidget->topLevelItemCount();i++)
    {
        // analisar todos os os modelos do gazebo existentes no mundo
        if(ui->treeWidget->topLevelItem(i)->text(0)=="Include")
        {
            // adquire a URL onde estão armazenados todos os modelos de vant do ambiente de simulação
            AppSettings settings;
            QString env = settings.getGazeboModelPath();
            QDir dir(env);

            // Listando Modelos
            QFileInfoList files = dir.entryInfoList();
            foreach (QFileInfo file, files)
            {
                if (file.isDir())
                {
                    // determinou-se que o terceiro include, corresponde sempre ao modelo do vant
                    if(file.fileName() == ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://",""))
                    {
                         env = env +
                                 "/" +
                                 ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://","") +
                                "/config/config.xml";

                         // informa o arquivo de configuração da malha de controle configurada na interface de simulação
                         // para dado vant (arquivo se ecnontra dentro da pasta config com o nome config.xml)
                         std::string command("export TILT_CONFIG="+env.toStdString());
                         std::system(command.c_str());
                    }
                }
            }
        }
    }

    QString base;
    if(hil)
    {
        std::system("kill `pgrep gzclient`");
        std::system("kill  `pgrep gzserver`");

        // verificar se o dispositivo está conectado em /dev/ttyUSB0
        int open_result = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        if(open_result == -1)
        {
            qDebug() << "Port Failed to Open";
            exit(EXIT_FAILURE);
        }
        else
        {
            std::system("setserial /dev/ttyUSB0 low_latency");
        }
        base = QString("hil.launch");
    }
    else base = QString ("gazebo.launch");

    if(istemplate) // caso seja template, o usuário é obrigado a adicionar informações em um arquivo .world
    {
        if(SaveAs())
        {
            mundo.Write(ui->treeWidget); // salvar modificações no mesmo arquivo de destino
            roslaunch::WriteNew(QString::fromStdString(mundo.word->Filename),base,hil); // configurando rotina de inicilização do simulador
            base = "xterm -e roslaunch Database " + base + " &";
            std::system(base.toStdString().c_str()); // comando de inicializar simulação
        }
    }
    else
    {
        mundo.Write(ui->treeWidget); // salvar modificações no mesmo arquivo de destino
        roslaunch::WriteNew(QString::fromStdString(mundo.word->Filename),base,hil); // configurando rotina de inicilização do simulador
        base = "xterm -e roslaunch Database " + base + " &";
        std::system(base.toStdString().c_str()); // comando de inicializar simulação
    }

    ui->jointValuesPushButton->setDisabled(false);
}

void MainWindow::on_actionNew_triggered()
{
    hil = false;
    // Diretório onde está os templates
    AppSettings settings;
    QString env = settings.getTiltProjectPath();
    env = env + "/worlds/templates";

    // Abertura de caixa de diálogo para selecionar arquivo template
    QString filename = QFileDialog::getOpenFileName(this
                                                    ,tr("New World")
                                                    , env.toStdString().c_str()
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
        QGraphicsScene * scene = new QGraphicsScene(ui->worldGraphicsView);
        QImage image(imagefile);
        scene->addPixmap(QPixmap::fromImage(image));
        ui->worldGraphicsView->setScene(scene);
        ui->worldGraphicsView->show();
    }

    ui->treeWidget->clear();
    // lê dados do arquivo .tpl informado anteriormnete
    mundo.getFirst(filename.toStdString(),ui->treeWidget);

    // habilitando funções de menuu e inicialização da interface
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->startGazeboPushButton->setEnabled(true);

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
        AppSettings settings;
        QString env = settings.getGazeboModelPath();
        QDir dir(env);
        QStringList list;
        list = item->text(1).split("//");
        if (!dir.cd(QString::fromStdString(list.at(1).toStdString()))) // "/tmp"
        {
            // não faz nada
        }
        else
        {
            // arquivo de descrição física
            std::string localmodel = env.toStdString()+"/"+list.at(1).toStdString()+"/robot/model.sdf";
            // arquivo de configuração de controlador
            std::string localconfig = env.toStdString()+"/"+list.at(1).toStdString()+"/config/config.xml";
            // abrir janela
            ModelSetupDialog modelsetup;
            modelsetup.setModel(localmodel,localconfig);
            modelsetup.setModal(true);
            modelsetup.exec();
            hil = modelsetup.hil;
            QString s = QString::number(hil);

            if(hil)
            {
                // Setando valores padronizados da simumlação para permitir bom
                // funcionamento do HIL
                for(int i = 0; i< ui->treeWidget->topLevelItemCount();i++)
                {
                    QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
                    if(item->text(0)=="Physics")
                    {
                        std::string description = "0.004"; // step time
                        item->child(1)->setText(1, QString::fromStdString(description));
                        std::string description2 = "1"; // fator de tempo real
                        item->child(2)->setText(1, QString::fromStdString(description2));
                        std::string description3 = "250"; // update
                        item->child(3)->setText(1, QString::fromStdString(description3));

                    }
                    if(item->text(0)=="Plugin")
                    {
                        item->child(2)->setText(1, QString("hil"));
                    }
                }

            }
            else
            {
                // Setando valores padronizados da simumlação para permitir bom
                // funcionamento do HIL
                for(int i = 0; i< ui->treeWidget->topLevelItemCount();i++)
                {
                    QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
                    /*if(item->text(0)=="Physics")
                    {
                        std::string description = "0.004"; // step time
                        item->child(1)->setText(1, QString::fromStdString(description));
                        std::string description2 = "1"; // fator de tempo real
                        item->child(2)->setText(1, QString::fromStdString(description2));
                        std::string description3 = "250"; // update
                        item->child(3)->setText(1, QString::fromStdString(description3));

                    }*/
                    if(item->text(0)=="Plugin")
                    {
                        item->child(2)->setText(1, QString("nothil"));
                    }
                }
            }

        }
    }
}


void MainWindow::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{
    Q_UNUSED(column)
    if(item->text(0) == "uri")
    {
        AppSettings settings;
        QString env = settings.getGazeboModelPath();
        QDir dir(env);
        QStringList list;
        list = item->text(1).split("//");
        // não há arquivo
        if (!dir.cd(QString::fromStdString(list.at(1).toStdString()))) // "/tmp"
        {
            QGraphicsScene * scene = new QGraphicsScene(ui->modelGraphicsView);
            ui->modelGraphicsView->setScene(scene);
            ui->modelGraphicsView->show();
        }
        else // há arquivo
        {
            // plotando foto do vant
            QString imagefile(env+"/"+list.at(1)+"/imagem.gif");
            QFile ff(imagefile);
            QFileInfo fileInfo(ff);
            if (fileInfo.exists())
            {
                QGraphicsScene * scene = new QGraphicsScene(ui->modelGraphicsView);
                QImage image(imagefile);
                scene->addPixmap(QPixmap::fromImage(image));
                ui->modelGraphicsView->setScene(scene);
                ui->modelGraphicsView->show();
            }
        }
    }
}

void MainWindow::on_actionOpen_triggered()
{
    AppSettings settings;
    QString env = settings.getProvantDatabasePath();
    env = env + "/worlds/worlds";
    // escolhe do arquivo .world
    QString filename = QFileDialog::getOpenFileName(this
                                                        ,tr("Open World")
                                                        , env.toStdString().c_str()
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
        QGraphicsScene * scene = new QGraphicsScene(ui->worldGraphicsView);
        QImage image(imagefile);
        scene->addPixmap(QPixmap::fromImage(image));
        ui->worldGraphicsView->setScene(scene);
        ui->worldGraphicsView->show();
    }
    // limpando árvore para adicionar novos dados
    ui->treeWidget->clear();
    // adicionando dados na árvore de dados
    mundo.getFirst(filename.toStdString(),ui->treeWidget);
    // habilitando funções de menuu e inicialização da interface

    hil = false;

    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->startGazeboPushButton->setEnabled(true);
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

    mundo.word->Filename = filename.toStdString();
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

void MainWindow::on_actionNewModel_triggered()
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

void MainWindow::on_actionAbout_triggered()
{
    AboutDialog newform(this);
    newform.setModal(true);
    newform.exec();
}

void MainWindow::on_jointValuesPushButton_clicked()
{
    JointsDialog newform(this);
    newform.setModal(true);
    newform.exec();
    ui->jointValuesPushButton->setDisabled(true);
}

void MainWindow::on_actionOptions_triggered()
{
    ApplicationSettingsDialog dialog(this);
    dialog.exec();
}
