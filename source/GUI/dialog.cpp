#include "dialog.h"
#include "ui_dialog.h"
#include"qdebug.h"
#include"Business/treeitens.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);
}

Dialog::~Dialog()
{
    delete ui;
}


void Dialog::setModel(std::string modelfile,std::string controllerfile)
{
    int j = 0; // variável para armezanar estratégia de controle corrente
    int i = 0; // variável para varrer diretório

    // obtém modelo e controlador armazenados em arquivos
    model.getFirst(modelfile,ui->treeWidget);
    controller.get(controllerfile,ui->listWidget_2,ui->listWidget);

    // adiciona no combobox todos os controladores existentes
    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        QDir dir(env.c_str());
        QFileInfoList files = dir.entryInfoList();
        ui->comboBox->clear();
        foreach (QFileInfo file, files)
        {
            if (file.isDir())
            {
                if(file.fileName().size()>2)
                {
                    ui->comboBox->addItem(file.fileName());
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
        ui->comboBox->setCurrentIndex(j);
    }

    // adicionar demais informaçõeos à janela
    ui->SampleEdit->setText(QString::fromStdString(controller.config->GetSampleTime()));
    ui->ErrorEdit->setText(QString::fromStdString(controller.config->GetLogErro()));
    ui->ActuatorEdit->setText(QString::fromStdString(controller.config->GetLogOut()));
    ui->SensorEdit->setText(QString::fromStdString(controller.config->GetLogIn()));
    ui->ReferenceEdit->setText(QString::fromStdString(controller.config->GetLogRef()));
}

void Dialog::on_pushButton_5_clicked()
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
        ui->comboBox->clear();
        foreach (QFileInfo file, files)
        {
            if (file.isDir())
            {
                if(file.fileName().size()>2)
                {
                    ui->comboBox->addItem(file.fileName());
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
        ui->comboBox->setCurrentIndex(j);
    }
}

void Dialog::on_pushButton_7_clicked()
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
        env = "cd "+env+"; catkin_make --pkg "+ ui->comboBox->currentText().toStdString() + " 2> OutCompile.txt; gedit OutCompile.txt";
        std::system(env.c_str());
    }
}

void Dialog::on_pushButton_6_clicked()
{
    // abrindo janela para abrir controaldor já existente
    DialogOpenController window;
    window.GetControllers();
    window.setModal(true);
    window.exec();
}

void Dialog::on_pushButton_3_clicked()
{
    // adicionando sensores
    QListWidgetItem* item = new QListWidgetItem(QString("New Sensor"));
    ui->listWidget_2->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

void Dialog::on_pushButton_4_clicked()
{
    // removendo sensor
    delete ui->listWidget_2->currentItem();
}

void Dialog::on_pushButton_clicked()
{
    // novo atuador na lista
    QListWidgetItem* item = new QListWidgetItem(QString("New Actuator"));
    ui->listWidget->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

void Dialog::on_pushButton_2_clicked()
{
    // deletendo da lista
    delete ui->listWidget->currentItem();
}


void Dialog::on_buttonBox_accepted()
{
     SaveConfig();
}

void Dialog::SaveConfig()
{
    // armazenando estratégia deo controle
    controller.config->SetStrategy("lib"+ui->comboBox->currentText().toStdString()+".so");
    // armazenando período de amostragem
    controller.config->SetSampleTime(ui->SampleEdit->text().toStdString());
    // armazenando nome dos arquivos para armazenamento de dados de simulação
    controller.config->SetLog(ui->ErrorEdit->text().toStdString(),
                              ui->ReferenceEdit->text().toStdString(),
                              ui->ActuatorEdit->text().toStdString(),
                              ui->SensorEdit->text().toStdString());
    // adicionando sensores
    controller.config->Delete();
    for(int i = 0; i < ui->listWidget_2->count(); ++i)
    {
        controller.config->AddSensor(ui->listWidget_2->item(i)->text().toStdString());
    }
    // adicionando atuadores
    for(int i = 0; i < ui->listWidget->count(); ++i)
    {
        controller.config->AddActuator(ui->listWidget->item(i)->text().toStdString());
    }
    // escrevendo em arquivo
    controller.config->WriteFile();
}
