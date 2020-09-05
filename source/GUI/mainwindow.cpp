/**
  * @file mainwindow.cpp
  * @author Arthur Viana Lara
  * @date 18/05/2018
  * Projeto: ProVANT
  */

#include "mainwindow.h"

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QTreeWidgetItemIterator>

#include <fcntl.h>
#include <libusb-1.0/libusb.h>

#include "aboutdialog.h"
#include "applicationsettingsdialog.h"
#include "DataAccess/RosElements/roslaunch.h"
#include "dialognewmodel.h"
#include "jointsdialog.h"
#include "ProcessOutput/processoutputwindow.h"
#include "Utils/appsettings.h"
#include "modelsetupdialog.h"

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

    // Verifica se todas as configurações de parametros está correta e em caso
    // contrário, exibe mensagens de erro para o usuaŕio
    AppSettings settings;
    if(settings.checkAllParametersSet()) {
        settings.applyValuesToEnvironmentVariables();
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_startGazeboPushButton_clicked()
{
    // Get a list with all the models available under gazeboModelPath
    AppSettings settings;
    QString gazeboModelPath = settings.getGazeboModelPath();
    QDir gazeboModelPathDir(gazeboModelPath);

    // Checks if gazebo model is valid, if it isn't report the error to the
    // user and stops the execution of this method.
    if(!gazeboModelPathDir.exists())
    {
        QMessageBox::critical(
                    this,
                    tr("Error"),
                    tr("An error ocurred while reading the gazebo model path, "
                       "and therefore it is not possible to start the "
                       "simulation. Please fix the configure value of the "
                       "gazebo model path and try again."));
        qCritical() << "An error ocurred when trying to start the gazebo "
                       "simulation. The path to the gazebo model path was "
                       "invalid and the startup process was halted.";
        return;
    }

    // Get a list of all the directories under gazeboModelPath
    QFileInfoList gazeboModels = gazeboModelPathDir.entryInfoList(
                QDir::NoDotAndDotDot | QDir::Dirs);

    // A string with the path of the model config file path.
    QString modelConfigFilePath;

    // Find the include values in the tree widget
    for(int i=0;i<ui->treeWidget->topLevelItemCount();i++)
    {
        if(ui->treeWidget->topLevelItem(i)->text(0) == "Include")
        {
            // Fins the item with the uri
            QString modelName;
            QTreeWidgetItem *currentItem = ui->treeWidget->topLevelItem(i);
            for(int i = 0; i < currentItem->childCount(); i++)
            {
                if(currentItem->child(i)->text(0) == "uri")
                {
                    modelName = currentItem->child(i)->text(1).replace(
                                "model://", "");
                    break;
                }
            }

            // If a model URI was not found, continues the loop to the next
            // element
            if(modelName.isEmpty())
            {
                continue;
            }

            // Loops trough the models list
            foreach (const QFileInfo &model, gazeboModels)
            {
                // Checks if the current model directory is equal to the
                // model managed by the tree widget item.
                if(model.fileName() == modelName)
                {
                    // Sets the path to the model config file and stops the
                    // loop
                    modelConfigFilePath = QDir::cleanPath(
                                gazeboModelPath +
                                QDir::separator() +
                                modelName +
                                QDir::separator() +
                                "config" +
                                QDir::separator() +
                                "config.xml");
                    break;
                }
            }

            // If the model was already found, finishes the loop
            if(!modelConfigFilePath.isEmpty())
            {
                break;
            }
        }
    }

    /*
     * If the modelConfigFilePath variable is empty, its because
     */
    if(modelConfigFilePath.isEmpty())
    {
        qCritical() << "An error ocurred while trying to start the gazebo "
                       "simulation. A valid model was not found under the "
                       "gazebo model path directory for the currently selected "
                       "world file.";
        QMessageBox::critical(
                    this,
                    tr("Error"),
                    tr("A model was not found under the gazebo model path "
                       "that matches any of the models specified in this "
                       "simulation world. Please check the values of the "
                       "configured models path and of the gazebo model path "
                       "setting and try again."));
        return;
    }

    // Stores the environment variable for the new simulation process
    QProcessEnvironment gazeboProcessEnv = settings.getEnvironmentVariables();

    // Checks if the model config file path is valid
    QFileInfo configFileInfo(modelConfigFilePath);
    if(configFileInfo.exists())
    {
        // Sets the TILT_CONFIG environment variable.
        if(!qputenv("TILT_CONFIG", modelConfigFilePath.toLocal8Bit()))
        {
            qCritical() << "An error ocurred while trying to start the gazebo "
                           "simulation. It was not possible to set the "
                           "TILT_CONFIG environment variable.";
            QMessageBox::critical(
                        this,
                        tr("Error"),
                        tr("A fatal error ocurred, it was not possible to set "
                           "the value of the TILT_CONFIG environment "
                           "variable. Check your user permissions and try "
                           "again."));
            gazeboProcessEnv.insert("TILT_CONFIG", modelConfigFilePath);
            return;
        }
    }
    else
    {
        qCritical() << "An error ocurred while trying to start the gazebo "
                       "simulation. The file in the specified path "
                    << modelConfigFilePath << " does not exist.";
        QMessageBox::critical(
                    this,
                    tr("Error"),
                    tr("The file %1 does not exist, please check the model "
                       "configuration and the value of the gazebo model path "
                       "setting and try again."));
        return;
    }

    QString launchFile;
    /*
     * OBS: This section of the code wont be refactored now.
     * I'm leaving it as is, note that a few modifications will be necessary
     * here to finish upgrading the use of system to QProcess.
     *
     * Further modification will be nedded to add a feature allowing the user
     * to select the USB port on the GUI.
     */
    if(hil)
    {
        std::system("kill `pgrep gzclient`");
        std::system("kill  `pgrep gzserver`");

        // verificar se o dispositivo está conectado em /dev/ttyUSB0
        int open_result = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        if(open_result == -1)
        {
            qCritical() << "Port Failed to Open";
            exit(EXIT_FAILURE);
        }
        else
        {
            std::system("setserial /dev/ttyUSB0 low_latency");
        }
        launchFile = QString("hil.launch");
    }
    else
    {
        launchFile = QString("gazebo.launch");
    }

    // Check if the currently opened file is a template.
    // If it is. Saves the file.
    if(istemplate)
    {
        if(!saveAs())
        {
            qCritical() << "Error, the template file must be saved before "
                           "running the simulation.";
            QMessageBox::critical(
                        this,
                        tr("Error"),
                        tr("The currently selected file is a template, so it "
                           "must be saved before running the simulation."));
            return;
        }
    }

    // Start the simulation.
    // Saves the modifications in  the simulation file.
    mundo.Write(ui->treeWidget);

    QString gazeboProcessCmd = "roslaunch";
    QStringList gazeboProcessArgs;
    gazeboProcessArgs << "Database" << launchFile;
    gazeboProcessArgs << QString("world:=%1")
                         .arg(QString::fromStdString(mundo.word->Filename));
    gazeboProcessArgs << QString("control_strategy:=%1")
                         .arg(modelConfigFilePath);

    // Creates the process to execute the simulation
    QProcess *gazeboSimulationProcess = new QProcess(this);
    gazeboSimulationProcess->setWorkingDirectory(
                settings.getCatkinWorkspacePath());
    gazeboSimulationProcess->setProgram(gazeboProcessCmd);
    gazeboSimulationProcess->setArguments(gazeboProcessArgs);
    gazeboSimulationProcess->setProcessEnvironment(gazeboProcessEnv);
    ProcessOutputWindow *gazeboProcessWindow = new ProcessOutputWindow(
                gazeboSimulationProcess, this);
    gazeboProcessWindow->setProcessName(tr("ProVANT Simulator Instance"));
    gazeboProcessWindow->show();
    gazeboProcessWindow->start(gazeboProcessEnv);
    qDebug() << "Started process id: " << gazeboSimulationProcess->processId();

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

void MainWindow::on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item,
                                                 int column)
{
    // permite edição de dados na segunda coluna da árvore de dados
    if (column == 1 && item->text(0) != "uri") {
            ui->treeWidget->editItem(item, column);
        }

    // If the field is of URI type, start the model editor.
    if(item->text(0) == "uri")
    {
        AppSettings settings;
        QString env = settings.getGazeboModelPath();
        QDir dir(env);
        QStringList list;
        list = item->text(1).split("//");

        // Model description file
        QString localModelPath = QDir::cleanPath(env +
                                                 QDir::separator() +
                                                 list.at(1) +
                                                 QDir::separator() +
                                                 "robot" +
                                                 QDir::separator() +
                                                 "model.sdf");

        // Controller configuration file path
        QString localConfig = QDir::cleanPath(
                    env + QDir::separator() + list.at(1) + QDir::separator() +
                    "config" + QDir::separator() + "config.xml");

        QFileInfo localConfigInfo(localConfig);
        QFileInfo modelFileInfo(localModelPath);

        if(!localConfigInfo.exists()) {
            QString modelName = getModelName(item);
            QMessageBox::critical(this,
                                  tr("Error"),
                                  tr("Error while trying to open the model "
                                     "config.xml file.\nPlease make sure that "
                                     "the model has a directory named config "
                                     "under its root directory and a config.xml "
                                     "file."));
            qCritical("%s%s. %s",
                      qUtf8Printable(tr("Error while trying to find the "
                                        "config.xml for the ")),
                      qUtf8Printable(modelName),
                      qUtf8Printable(tr("Please make sure this model exists "
                                        "and has a config.xml file.")));
            return;
        }
        if(!modelFileInfo.exists()) {
            QMessageBox::critical(this,
                                  tr("Error"),
                                  tr("Error while trying to open the SDF file "
                                     "for the selected model. Please make sure "
                                     "the file exists, is under the correct"
                                     " path and is named robot.sdf."));
            qCritical("Error while trying to open the SDF file for the "
                      "selected model. Please make sure this model exists.");
            return;
        }

        // Opens the window
        ModelSetupDialog modelsetup;
        modelsetup.setModel(localModelPath, localConfig);
        modelsetup.setModal(true);
        modelsetup.exec();

        hil = modelsetup.hil();
        QString s = QString::number(hil);

        if(hil)
        {

            // Setando valores padronizados da simumlação para permitir bom
            // funcionamento do HIL
            for(int i = 0; i< ui->treeWidget->topLevelItemCount();i++)
            {
                QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);

                if(item->text(0) == "Physics")
                {
                    QTreeWidgetItemIterator it(item);

                    while(*it)
                    {
                        // Set step time
                        if((*it)->text(0) == "Step time")
                        {
                            (*it)->setText(1, "0.004");
                        }
                        // Set the real time factor
                        else if((*it)->text(0) == "Real time factor")
                        {
                            (*it)->setText(1, "1");
                        }
                        // Set the real time update rate
                        else if((*it)->text(0) == "Real time update rate")
                        {
                            (*it)->setText(1, "250");
                        }
                        ++it;
                    }
                }
                else if(item->text(0) == "Plugin")
                {
                    //! @todo Check the behavior of this part with Arthur
                    //! This action was modified because it caused the
                    //! application to crash after closing the modelsetupdialog.
                    //item->child(2)->setText(1, QString("hil"));

                    QTreeWidgetItemIterator it(item);

                    while(*it)
                    {
                        if((*it)->text(0) == "ok")
                        {
                            (*it)->setText(1, "hil");
                        }

                        ++it;
                    }
                }
            }

        }
        else
        {
            // Disables the hil
            for(int i = 0; i< ui->treeWidget->topLevelItemCount();i++)
            {
                QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
                if(item->text(0)=="Plugin")
                {
                    QTreeWidgetItemIterator it(item);

                    while(*it)
                    {
                        if((*it)->text(0) == "ok")
                        {
                            (*it)->setText(1, "nothil");
                        }

                        ++it;
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
    QString worldsPackagePath = settings.getWorldsPackagePath();
    // Open dialog to select the world file
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Open World"),
                                                    worldsPackagePath,
                                                    tr("World Files (*.world)")
                                                    );
    // Check if the user cancelled the operation (i.e. the filename is empty)
    if(filename.isEmpty()) return;

    // Get the directory of the file
    QFileInfo worldFileInfo(filename);
    QString dir = worldFileInfo.absoluteDir().absolutePath();

    // Get the file path for the scenary image
    QString imagefile(QDir::cleanPath(dir + QDir::separator() + "imagem.gif"));
    QFileInfo imageFileInfo(imagefile);
    if (imageFileInfo.exists())
    {
        QGraphicsScene * scene = new QGraphicsScene(ui->worldGraphicsView);
        scene->addPixmap(QPixmap(imagefile));
        ui->worldGraphicsView->setScene(scene);
        ui->worldGraphicsView->show();
    }
    // Clean the display widget tree
    ui->treeWidget->clear();
    // Adds new world information
    mundo.getFirst(filename.toStdString(), ui->treeWidget);

    // Enables the menu itens
    hil = false;
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->startGazeboPushButton->setEnabled(true);
    istemplate = false;
}

bool MainWindow::saveAs()
{
    AppSettings settings;
    QString worldsFolderPath = settings.getWorldsPackagePath();

    // Open a dialog to let the user select the save file name
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save World"),
                                                    worldsFolderPath,
                                                    tr("World Files (*.world)")
                                                    );

    // Check if the user canceled the operation (i.e. the filename is empty)
    if(filename.isEmpty()) return false;

    // Ensure that the file is terminated with a .world suffix
    if(!filename.endsWith(".world")) {
        filename += ".world";
    }

    mundo.word->Filename = filename.toStdString();
    // Update file
    mundo.Write(ui->treeWidget);
    // Update template state
    istemplate = false;
    return true;
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::on_actionNewModel_triggered()
{
    DialogNewModel newmodel(ui,this);
    newmodel.setModal(true);
    newmodel.exec();
}

void MainWindow::on_actionSave_triggered()
{
    saveAs();
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

QString MainWindow::getModelName(QTreeWidgetItem *item)
{
    QTreeWidgetItem *parent = item->parent();
    if(parent != nullptr) {
        for(int i = 0; i < parent->childCount(); i++) {
            QTreeWidgetItem *child = parent->child(i);
            if(child->text(0).toLower() == "name") {
                return child->text(1);
            }
        }
    }
    return QString();
}
