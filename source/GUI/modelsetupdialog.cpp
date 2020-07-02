#include "modelsetupdialog.h"
#include "ui_modelsetupdialog.h"
#include"qdebug.h"
#include"Business/treeitens.h"

ModelSetupDialog::ModelSetupDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelSetupDialog)
{
    ui->setupUi(this);
    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);
}

ModelSetupDialog::~ModelSetupDialog()
{
    delete ui;
}


void ModelSetupDialog::setModel(std::string modelfile,std::string controllerfile)
{
    int j = 0; // variável para armezanar estratégia de controle corrente
    int i = 0; // variável para varrer diretório

    // obtém modelo e controlador armazenados em arquivos
    model.getFirst(modelfile,ui->treeWidget);
    controller.get(controllerfile,ui->sensorsListWidget,ui->actuatorsListWidget);

    // adiciona no combobox todos os controladores existentes
    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        QDir dir(env.c_str());
        QFileInfoList files = dir.entryInfoList();
        ui->controllerComboBox->clear();
        foreach (QFileInfo file, files)
        {
            if (file.isDir())
            {
                if(file.fileName().size()>2)
                {
                    ui->controllerComboBox->addItem(file.fileName());
                    QString temp = "lib"+file.fileName()+".so";
                    if(temp==QString::fromStdString(controller.config->GetStrategy()))
                    {
                        // item atual do combobox
                        j = i;
                    }
                    i++;
                }
            }
        }
        // adicionar item atual ao combobox
        ui->controllerComboBox->setCurrentIndex(j);
    }

    // adicionar demais informaçõeos à janela
    ui->SampleEdit->setText(QString::fromStdString(controller.config->GetSampleTime()));
    ui->ErrorEdit->setText(QString::fromStdString(controller.config->GetLogErro()));
    ui->ActuatorEdit->setText(QString::fromStdString(controller.config->GetLogOut()));
    ui->SensorEdit->setText(QString::fromStdString(controller.config->GetLogIn()));
    ui->ReferenceEdit->setText(QString::fromStdString(controller.config->GetLogRef()));
}

void ModelSetupDialog::on_newControllerButton_clicked()
{
    int j = 0; // variável para armezanar estratégia de controle corrente
    int i = 0; // variável para varrer diretório

    // abrindo janela para usuário determinar o nome do projeto
    DialogNewController window;
    window.setModal(true);
    window.exec();

    // colocando novo projeto de controle no listbox
    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        QDir dir(env.c_str());
        // obtendo o nome de todos o projetos de estratégias de controle
        QFileInfoList files = dir.entryInfoList();
        ui->controllerComboBox->clear();
        foreach (QFileInfo file, files)
        {
            if (file.isDir())
            {
                if(file.fileName().size()>2)
                {
                    ui->controllerComboBox->addItem(file.fileName());
                    QString temp = "lib"+file.fileName()+".so";
                    if(temp==QString::fromStdString(controller.config->GetStrategy()))
                    {
                        // item atual do combobox
                        j = i;
                    }
                    i++;
                }
            }
        }
        ui->controllerComboBox->setCurrentIndex(j);
    }
}

void ModelSetupDialog::on_compileControllerButton_clicked()
{
    // obtendo caminho da pasta catkin_ws
    char const* tmp = getenv( "DIR_ROS" );
    if ( tmp == NULL )
    {
        qDebug() << "Problemas com variavel de ambiente ";
    }
    else
    {
        // comando para compilação e armazenamento da saída de erro em arquivo texto
        std::string env(tmp);
        env = "cd "+env+"; catkin_make --pkg "+ ui->controllerComboBox->currentText().toStdString() + " 2> OutCompile.txt; xdg-open OutCompile.txt";
        qDebug() << env.c_str();
        std::system(env.c_str());
    }
}

void ModelSetupDialog::on_openControllerButton_clicked()
{
    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        std::string command("xdg-open "+env+"/"+ui->controllerComboBox->currentText().toStdString());
        std::system(command.c_str());
    }
}

void ModelSetupDialog::on_addSensorButton_clicked()
{
    // adicionando sensores
    QListWidgetItem* item = new QListWidgetItem(QString("New Sensor"));
    ui->sensorsListWidget->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

void ModelSetupDialog::on_removeSensorButton_clicked()
{
    // removendo sensor
    delete ui->sensorsListWidget->currentItem();
}

void ModelSetupDialog::on_addActuatorButton_clicked()
{
    // novo atuador na lista
    QListWidgetItem* item = new QListWidgetItem(QString("New Actuator"));
    ui->actuatorsListWidget->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

void ModelSetupDialog::on_removeActuatorButton_clicked()
{
    // deletendo da lista
    delete ui->actuatorsListWidget->currentItem();
}


void ModelSetupDialog::on_buttonBox_accepted()
{
     SaveConfig();
}

void ModelSetupDialog::SaveConfig()
{
    // armazenando estratégia deo controle
    controller.config->SetStrategy("lib"+ui->controllerComboBox->currentText().toStdString()+".so");
    // armazenando período de amostragem
    controller.config->SetSampleTime(ui->SampleEdit->text().toStdString());
    // armazenando nome dos arquivos para armazenamento de dados de simulação
    controller.config->SetLog(ui->ErrorEdit->text().toStdString(),
                              ui->ReferenceEdit->text().toStdString(),
                              ui->ActuatorEdit->text().toStdString(),
                              ui->SensorEdit->text().toStdString());
    // adicionando sensores
    controller.config->Delete();
    for(int i = 0; i < ui->sensorsListWidget->count(); ++i)
    {
        controller.config->AddSensor(ui->sensorsListWidget->item(i)->text().toStdString());
    }
    // adicionando atuadores
    for(int i = 0; i < ui->actuatorsListWidget->count(); ++i)
    {
        controller.config->AddActuator(ui->actuatorsListWidget->item(i)->text().toStdString());
    }
    // escrevendo em arquivo
    controller.config->WriteFile();
}


void ModelSetupDialog::on_hilCheckBox_clicked(bool checked)
{
    if(checked==true)
    {
        ui->SampleEdit->setEnabled(false);
        ui->ErrorEdit->setEnabled(false);
        ui->ActuatorEdit->setEnabled(false);
        ui->SensorEdit->setEnabled(false);
        ui->ReferenceEdit->setEnabled(false);
        ui->newControllerButton->setEnabled(false);
        ui->openControllerButton->setEnabled(false);
        ui->compileControllerButton->setEnabled(false);
        ui->controllerComboBox->setEnabled(false);
        hil = true;
    }
    else
    {
        ui->SampleEdit->setEnabled(true);
        ui->ErrorEdit->setEnabled(true);
        ui->ActuatorEdit->setEnabled(true);
        ui->SensorEdit->setEnabled(true);
        ui->ReferenceEdit->setEnabled(true);
        ui->newControllerButton->setEnabled(true);
        ui->openControllerButton->setEnabled(true);
        ui->compileControllerButton->setEnabled(true);
        ui->controllerComboBox->setEnabled(true);
        hil = false;
    }
}
