#include "jointsdialog.h"
#include "ui_jointsdialog.h"
#include "qdebug.h"
#include <fstream>
#include "Business/treeitens.h"

JointsDialog::JointsDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::JointsDialog)
{
  ui->setupUi(this);

  std::system("/home/arthur/catkin_ws/src/ProVANT-Simulator_Developer/source/"
              "GUI/Shell/getjoints > output.txt");

  std::ifstream infile("output.txt");
  ui->treeWidget->setColumnCount(2);

  std::string a;
  while (infile >> a)
  {
    QTreeWidgetItem* item = new QTreeWidgetItem(ui->treeWidget);
    item->setText(0, QString::fromStdString(a));
    item->setText(1, QString::fromStdString("0"));
    item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
    ui->treeWidget->addTopLevelItem(item);
  }
  const QString name("Initial Joint Setup");
  setWindowTitle(name);
}

JointsDialog::~JointsDialog()
{
  delete ui;
}

void JointsDialog::on_buttonBox_accepted()
{
  for (int i = 0; i < ui->treeWidget->topLevelItemCount(); i++)
  {
    QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
    std::string command = "rosservice call /gazebo/set_model_configuration "
                          "'{model_name: \"newmodel\", joint_names:['" +
                          item->text(0).toStdString() +
                          "'], joint_positions:[" +
                          item->text(1).toStdString() + "]}'";
    // qDebug() << QString::fromStdString(command);
    std::system(command.c_str());
  }
}

void JointsDialog::on_treeWidget_itemDoubleClicked(QTreeWidgetItem* item,
                                                   int column)
{
  // permite edição apenas na segunda coluna
  if (column == 1)
  {
    ui->treeWidget->editItem(item, column);
  }
}
