#include "aboutdialog.h"
#include "ui_aboutdialog.h"

AboutDialog::AboutDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::AboutDialog)
{
  ui->setupUi(this);

  QPixmap provant(":/logos/logos/logo_provant.svg");
  int w = ui->provantLogo->width();
  int h = ui->provantLogo->height();
  ui->provantLogo->setPixmap(provant.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap ufmg(":/logos/logos/ufmg.png");
  w = ui->ufmgLogo->width();
  h = ui->ufmgLogo->height();
  ui->ufmgLogo->setPixmap(ufmg.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap ufsc(":/logos/logos/ufsc.svg");
  w = ui->ufscLogo->width();
  h = ui->ufscLogo->height();
  ui->ufscLogo->setPixmap(ufsc.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap sevilla(":/logos/logos/sevilla.png");
  w = ui->sevilleLogo->width();
  h = ui->sevilleLogo->height();
  ui->sevilleLogo->setPixmap(sevilla.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap cnpq(":/logos/logos/CNPq.png");
  w = ui->cnpqLogo->width();
  h = ui->cnpqLogo->height();
  ui->cnpqLogo->setPixmap(cnpq.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap capes(":/logos/logos/CAPES.png");
  w = ui->capesLogo->width();
  h = ui->capesLogo->height();
  ui->capesLogo->setPixmap(capes.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap fapemig(":/logos/logos/Fapemig.png");
  w = ui->fapemigLogo->width();
  h = ui->fapemigLogo->height();
  ui->fapemigLogo->setPixmap(fapemig.scaled(w, h, Qt::KeepAspectRatio));

  QPixmap insac(":/logos/logos/insac.png");
  w = ui->insacLogo->width();
  h = ui->insacLogo->height();
  ui->insacLogo->setPixmap(insac.scaled(w, h, Qt::KeepAspectRatio));
}

AboutDialog::~AboutDialog()
{
  delete ui;
}
