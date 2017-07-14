#include "dialogsensors.h"
#include "ui_dialogsensors.h"

#include"dialognfields.h"
#include"dialogsonar.h"
#include"dialoggps.h"
#include"dialogimu.h"
#include"dialogmag.h"

DialogSensors::DialogSensors(Model model,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogSensors)
{
    ui->setupUi(this);
    registry = model;
}

DialogSensors::~DialogSensors()
{
    delete ui;
}

void DialogSensors::on_buttonBox_accepted()
{
    if(ui->comboBox->currentText()=="Model plugin")
    {
        DialogNfields dial;
        dial.setModal(true);
        dial.exec();
        n = dial.Nfields();
        type = "Model Plugin";
    }
    if(ui->comboBox->currentText()=="sonar")
    {
        DialogSonar sonar(registry);
        sonar.setModal(true);
        sonar.exec();
        values.push_back(sonar.max);
        values.push_back(sonar.min);
        values.push_back(sonar.radius);
        values.push_back(sonar.link);
        values.push_back(sonar.topic);
        type = "sonar";
    }
    if(ui->comboBox->currentText()=="imu")
    {
        DialogIMU imu(registry);
        imu.setModal(true);
        imu.exec();

        values.push_back(imu.link);
        values.push_back(imu.topic);
        values.push_back(imu.noise);
        type = "imu";
    }
    if(ui->comboBox->currentText()=="gps")
    {
        DialogGps gps(registry);
        gps.setModal(true);
        gps.exec();

        values.push_back(gps.link);
        values.push_back(gps.topic);
        values.push_back(gps.noise);
        type = "gps";
    }
    if(ui->comboBox->currentText()=="magnetometer")
    {
        DialogMag mag(registry);
        mag.setModal(true);
        mag.exec();

        values.push_back(mag.link);
        values.push_back(mag.topic);
        values.push_back(mag.noise);
        type = "mag";
    }
}
