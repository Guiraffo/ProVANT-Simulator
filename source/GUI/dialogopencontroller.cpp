#include "dialogopencontroller.h"
#include "ui_dialogopencontroller.h"

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
    char const* tmp = getenv( "TILT_PROJECT" );
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
    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        std::string command("nautilus "+env+"/"+ui->comboBox->currentText().toStdString());
        std::system(command.c_str());
    }
}
