#include "aboutform.h"
#include "ui_aboutform.h"

AboutForm::AboutForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AboutForm)
{
    ui->setupUi(this);
    QPixmap provant("/home/arthur/catkin_ws/src/ProVANT-Simulator/source/GUI/logos/provant_ufmg_ufsc.jpg");
    ui->label->setPixmap(provant);
}

AboutForm::~AboutForm()
{
    delete ui;
}
