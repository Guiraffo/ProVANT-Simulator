#include "dialognfields.h"
#include "ui_dialognfields.h"

DialogNfields::DialogNfields(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogNfields)
{
    ui->setupUi(this);
    n = 0;
}

DialogNfields::~DialogNfields()
{
    delete ui;
}

void DialogNfields::on_buttonBox_accepted()
{
    n = ui->lineEdit->text().toFloat();
}

int DialogNfields::Nfields()
{
    return n;
}
