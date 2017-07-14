#include "dialognewcontroller.h"
#include "ui_dialognewcontroller.h"
#include "qdebug.h"

DialogNewController::DialogNewController(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogNewController)
{
    ui->setupUi(this);
}

DialogNewController::~DialogNewController()
{
    delete ui;
}

void DialogNewController::on_buttonBox_accepted()
{
    newstrategy::CreateProject(ui->lineEdit->text());
}
