#include "dialogopencontroller.h"
#include "ui_dialogopencontroller.h"

#include "Utils/appsettings.h"

DialogOpenController::DialogOpenController(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogOpenController)
{
    ui->setupUi(this);
}

DialogOpenController::~DialogOpenController()
{
    delete ui;
}

void DialogOpenController::GetControllers()
{
    AppSettings settings;
    char const* tmp = settings.getTiltProjectPath().toStdString().c_str();
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        QDir dir(env.c_str());
        QFileInfoList files = dir.entryInfoList();
        foreach (QFileInfo file, files)
        {
            if (file.isDir())
            {
                if(file.fileName().size()>2)
                {
                    ui->comboBox->addItem(file.fileName());
                }
            }
        }
    }
}


void DialogOpenController::on_buttonBox_accepted()
{
    AppSettings settings;
    char const* tmp = settings.getTiltProjectPath().toStdString().c_str();
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        std::string command("nautilus "+env+"/"+ui->comboBox->currentText().toStdString());
        std::system(command.c_str());
    }
}
