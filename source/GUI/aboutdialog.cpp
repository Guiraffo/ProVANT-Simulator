#include "aboutdialog.h"
#include "ui_aboutdialog.h"

AboutDialog::AboutDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AboutDialog)
{
    ui->setupUi(this);

    char* tmp = getenv( "TILT_PROJECT" );

    QString env(tmp);
    QString path("/source/GUI/logos/");
    QString fullpath = env+path;

    QPixmap provant(fullpath+"provant_ufmg_ufsc.jpg");
    int w = ui->label_2->width();
    int h = ui->label_2->height();
    ui->label_2->setPixmap(provant.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap ufmg(fullpath+"ufmg.png");
    w = ui->label_3->width();
    h = ui->label_3->height();
    ui->label_3->setPixmap(ufmg.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap ufsc(fullpath+"ufsc.svg");
    w = ui->label_4->width();
    h = ui->label_4->height();
    ui->label_4->setPixmap(ufsc.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap sevilla(fullpath+"sevilla.png");
    w = ui->label_5->width();
    h = ui->label_5->height();
    ui->label_5->setPixmap(sevilla.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap cnpq(fullpath+"CNPq.png");
    w = ui->label_6->width();
    h = ui->label_6->height();
    ui->label_6->setPixmap(cnpq.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap capes(fullpath+"CAPES.png");
    w = ui->label_7->width();
    h = ui->label_7->height();
    ui->label_7->setPixmap(capes.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap fapemig(fullpath+"Fapemig.png");
    w = ui->label_8->width();
    h = ui->label_8->height();
    ui->label_8->setPixmap(fapemig.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap insac(fullpath+"insac.jpeg");
    w = ui->label_9->width();
    h = ui->label_9->height();
    ui->label_9->setPixmap(insac.scaled(w,h,Qt::KeepAspectRatio));


}

AboutDialog::~AboutDialog()
{
    delete ui;
}
