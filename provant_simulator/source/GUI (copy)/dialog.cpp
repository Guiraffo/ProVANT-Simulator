#include "dialog.h"
#include "ui_dialog.h"
#include"qdebug.h"
#include"Business/treeitens.h"
#include"dialogsensors.h"

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
    model.getFirst(modelfile,ui->treeWidget);
    controller.get(controllerfile,ui->listWidget_2,ui->listWidget);

    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        QDir dir(env.c_str());
        QFileInfoList files = dir.entryInfoList();
        int i = 0;
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
                        i++;
                    }
                }
            }
        }
        ui->comboBox->setCurrentIndex(i);
    }

    ui->SampleEdit->setText(QString::fromStdString(controller.config->GetSampleTime()));
    ui->ErrorEdit->setText(QString::fromStdString(controller.config->GetLogErro()));
    ui->ActuatorEdit->setText(QString::fromStdString(controller.config->GetLogOut()));
    ui->SensorEdit->setText(QString::fromStdString(controller.config->GetLogIn()));
    ui->ReferenceEdit->setText(QString::fromStdString(controller.config->GetLogRef()));
}

void Dialog::on_comboBox_activated(const QString &arg1)
{

}

void Dialog::on_pushButton_5_clicked()
{
    DialogNewController window;
    window.setModal(true);
    window.exec();

    char const* tmp = getenv( "TILT_PROJECT" );
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        env = env + "/source/Structure/control_strategies";
        QDir dir(env.c_str());
        QFileInfoList files = dir.entryInfoList();
        int i = 0;
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
                        i++;
                    }
                }
            }
        }
        ui->comboBox->setCurrentIndex(i);
    }
}

void Dialog::on_pushButton_7_clicked()
{

    char const* tmp = getenv( "DIR_ROS" );
    if ( tmp == NULL )
    {
        qDebug() << "Problemas com variavel de ambiente ";
    }
    else
    {
        std::string env(tmp);
        env = "cd "+env+"; catkin_make --pkg "+ ui->comboBox->currentText().toStdString() + " 2> OutCompile.txt; gedit OutCompile.txt";
        std::system(env.c_str());
    }
}

void Dialog::on_pushButton_6_clicked()
{
    DialogOpenController window;
    window.GetControllers();
    window.setModal(true);
    window.exec();
}

void Dialog::on_treeWidget_doubleClicked(const QModelIndex &index)
{

}

void Dialog::on_pushButton_3_clicked()
{
    QListWidgetItem* item = new QListWidgetItem(QString("New Sensor"));
    ui->listWidget_2->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

void Dialog::on_pushButton_4_clicked()
{
    delete ui->listWidget_2->currentItem();
}

void Dialog::on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column)
{
    if (column == 1) {
            ui->treeWidget->editItem(item, column);
        }
    if (column == 0 && item->parent()->text(0)=="Plugin")
    {
        if(item->text(0)!="Name"||item->text(0)!="Filename")
        {
            ui->treeWidget->editItem(item, column);
        }
    }
}

void Dialog::on_pushButton_clicked()
{
    QListWidgetItem* item = new QListWidgetItem(QString("New Actuator"));
    ui->listWidget->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

void Dialog::on_pushButton_2_clicked()
{
    delete ui->listWidget->currentItem();
}

void Dialog::on_pushButton_9_clicked()
{
    if(ui->treeWidget->currentItem()->text(0)=="Plugin")
    {
        delete ui->treeWidget->currentItem();
    }
    if(ui->treeWidget->currentItem()->text(0)=="Sensor")
    {
        delete ui->treeWidget->currentItem();
    }
}

void Dialog::on_buttonBox_accepted()
{
     SaveConfig();
     SaveParameters();
}

void Dialog::SaveConfig()
{
    controller.config->SetStrategy("lib"+ui->comboBox->currentText().toStdString()+".so");
    controller.config->SetSampleTime(ui->SampleEdit->text().toStdString());
    controller.config->SetLog(ui->ErrorEdit->text().toStdString(),
                              ui->ReferenceEdit->text().toStdString(),
                              ui->ActuatorEdit->text().toStdString(),
                              ui->SensorEdit->text().toStdString());
    controller.config->Delete();
    for(int i = 0; i < ui->listWidget_2->count(); ++i)
    {
        controller.config->AddSensor(ui->listWidget_2->item(i)->text().toStdString());
    }
    for(int i = 0; i < ui->listWidget->count(); ++i)
    {
        controller.config->AddActuator(ui->listWidget->item(i)->text().toStdString());
    }

    controller.config->WriteFile();
}

void Dialog::SaveParameters()
{
    model.Write(ui->treeWidget);
}

void Dialog::on_pushButton_8_clicked()
{
    DialogSensors dial(model);
    dial.setModal(true);
    dial.exec();
    if(dial.type=="Model Plugin")
    {
        QTreeWidgetItem * item, *edit;
        item = TreeItens::AddRoot("Plugin","",ui->treeWidget);
        edit = TreeItens::AddChild(item,"Name","NewName");
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(item,"Filename","NewFilename");
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        for(int i = 0; i<dial.n;i++)
        {
            edit = TreeItens::AddChild(item,"NameParameter","NewDescription");
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        }
    }
    if(dial.type=="sonar")
    {
        QTreeWidgetItem* element = TreeItens::AddRoot("Sensor","",ui->treeWidget);
        TreeItens::AddChild(element,"Name","sonar");
        TreeItens::AddChild(element,"Type","sonar");
        TreeItens::AddChild(element,"Always_on","true");
        TreeItens::AddChild(element,"Visualize","true");
        TreeItens::AddChild(element,"Link",dial.values.at(3).toStdString());
        QTreeWidgetItem* edit = TreeItens::AddChild(element,"Pose","");
        model.splitvector("0 0 0 0 0 0",edit,true);
        edit = TreeItens::AddChild(element,"Update_rate","1000");
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Topic",dial.values.at(4).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Max",dial.values.at(0).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Min",dial.values.at(1).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Radius",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }
    if(dial.type=="imu")
    {
        QTreeWidgetItem* element = TreeItens::AddRoot("Sensor","",ui->treeWidget);
        TreeItens::AddChild(element,"Name","imu");
        TreeItens::AddChild(element,"Type","imu");
        TreeItens::AddChild(element,"Always_on","true");
        TreeItens::AddChild(element,"Visualize","true");
        TreeItens::AddChild(element,"Link",dial.values.at(0).toStdString());
        QTreeWidgetItem* edit = TreeItens::AddChild(element,"Pose","");
        model.splitvector("0 0 0 0 0 0",edit,true);
        edit = TreeItens::AddChild(element,"Update_rate","1000");
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Topic",dial.values.at(1).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QString zero("0");
        QTreeWidgetItem* accel = TreeItens::AddChild(element,"Acceleration","");
        QTreeWidgetItem* accelx = TreeItens::AddChild(accel,"x","");
        edit = TreeItens::AddChild(accelx,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelx,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelx,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelx,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelx,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelx,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        QTreeWidgetItem* accely = TreeItens::AddChild(accel,"y","");
        edit = TreeItens::AddChild(accely,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accely,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accely,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accely,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accely,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accely,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        QTreeWidgetItem* accelz = TreeItens::AddChild(accel,"z","");
        edit = TreeItens::AddChild(accelz,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelz,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelz,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelz,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelz,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(accelz,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        QTreeWidgetItem* ang = TreeItens::AddChild(element,"Angular","");
        QTreeWidgetItem* angx = TreeItens::AddChild(ang,"x","");
        edit = TreeItens::AddChild(angx,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angx,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angx,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angx,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angx,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angx,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        QTreeWidgetItem* angy = TreeItens::AddChild(ang,"y","");
        edit = TreeItens::AddChild(angy,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angy,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angy,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angy,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angy,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angy,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        QTreeWidgetItem* angz = TreeItens::AddChild(ang,"z","");
        edit = TreeItens::AddChild(angz,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angz,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angz,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angz,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angz,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(angz,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }
    if(dial.type=="gps")
    {
        QTreeWidgetItem* element = TreeItens::AddRoot("Sensor","",ui->treeWidget);
        TreeItens::AddChild(element,"Name","gps");
        TreeItens::AddChild(element,"Type","gps");
        TreeItens::AddChild(element,"Always_on","true");
        TreeItens::AddChild(element,"Visualize","true");
        TreeItens::AddChild(element,"Link",dial.values.at(0).toStdString());
        QTreeWidgetItem* edit = TreeItens::AddChild(element,"Pose","");
        model.splitvector("0 0 0 0 0 0",edit,true);
        edit = TreeItens::AddChild(element,"Update_rate","1000");
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Topic",dial.values.at(1).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QString zero("0");
        QTreeWidgetItem* pos = TreeItens::AddChild(element,"position sensing","");
        QTreeWidgetItem* pos_hor = TreeItens::AddChild(pos,"horizontal","");
        edit = TreeItens::AddChild(pos_hor,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_hor,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_hor,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_hor,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_hor,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_hor,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QTreeWidgetItem* pos_ver = TreeItens::AddChild(pos,"vertical","");
        edit = TreeItens::AddChild(pos_ver,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_ver,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_ver,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_ver,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_ver,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(pos_ver,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);


        QTreeWidgetItem* vel = TreeItens::AddChild(element,"velocity sensing","");
        QTreeWidgetItem* vel_hor = TreeItens::AddChild(vel,"horizontal","");
        edit = TreeItens::AddChild(vel_hor,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_hor,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_hor,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_hor,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_hor,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_hor,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QTreeWidgetItem* vel_ver = TreeItens::AddChild(vel,"vertical","");
        edit = TreeItens::AddChild(vel_ver,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_ver,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_ver,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_ver,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_ver,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(vel_ver,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }
    if(dial.type=="mag")
    {
        QTreeWidgetItem* element = TreeItens::AddRoot("Sensor","",ui->treeWidget);
        TreeItens::AddChild(element,"Name","magnetometer");
        TreeItens::AddChild(element,"Type","magnetometer");
        TreeItens::AddChild(element,"Always_on","true");
        TreeItens::AddChild(element,"Visualize","true");
        TreeItens::AddChild(element,"Link",dial.values.at(0).toStdString());
        QTreeWidgetItem* edit = TreeItens::AddChild(element,"Pose","");
        model.splitvector("0 0 0 0 0 0",edit,true);
        edit = TreeItens::AddChild(element,"Update_rate","1000");
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(element,"Topic",dial.values.at(1).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QString zero("0");
        QTreeWidgetItem* X = TreeItens::AddChild(element,"x","");
        edit = TreeItens::AddChild(X,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(X,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(X,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(X,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(X,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(X,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QTreeWidgetItem* Y = TreeItens::AddChild(element,"y","");
        edit = TreeItens::AddChild(Y,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Y,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Y,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Y,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Y,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Y,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

        QTreeWidgetItem* Z = TreeItens::AddChild(element,"z","");
        edit = TreeItens::AddChild(Z,"type",dial.values.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Z,"mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Z,"stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Z,"bias mean",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Z,"bias stddev",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Z,"precision",zero.toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }


}
