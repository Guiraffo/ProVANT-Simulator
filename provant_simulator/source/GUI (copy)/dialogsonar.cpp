#include "dialogsonar.h"
#include "ui_dialogsonar.h"


DialogSonar::DialogSonar(Model model,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogSonar)
{
    ui->setupUi(this);
    registry = model;
    std::vector<link_DA> data = registry.actualmodel->model.GetListsLinks();

    for(uint i = 0;i<data.size();i++)
    {
        ui->comboBox->addItem(QString::fromStdString(data.at(i).name));
    }
}

DialogSonar::~DialogSonar()
{
    delete ui;
}

void DialogSonar::on_buttonBox_accepted()
{
    max = ui->lineEdit->text();
    min = ui->lineEdit_2->text();
    radius = ui->lineEdit_3->text();
    topic = ui->lineEdit_4->text();
    link = QString::number(ui->comboBox->currentIndex());
}


