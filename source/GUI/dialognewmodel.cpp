#include "dialognewmodel.h"
#include "ui_dialognewmodel.h"

#include "Business/treeitens.h"
#include "Utils/appsettings.h"
#include "Utils/modellist.h"

#include <QDebug>
#include <QDir>
#include <QMessageBox>
#include <QString>

DialogNewModel::DialogNewModel(Ui::MainWindow* mainWindow, QWidget* parent)
  : QDialog(parent), ui(new Ui::Dialognewmodel), parentUi(mainWindow)
{
  ui->setupUi(this);
  populateModelList();
}

DialogNewModel::~DialogNewModel()
{
  delete ui;
}

void DialogNewModel::populateModelList()
{
  const ModelList models{};
  foreach (const auto model, models.models())
  {
    ui->comboBox->addItem(model);
  }
}

void DialogNewModel::on_buttonBox_accepted()
{
  // Add the new model to the tree widget item of the world configuration
  QTreeWidgetItem *element, *elementPose, *edit;

  element = addRoot("Include", "", parentUi->treeWidget);
  edit = addChild(element, "name", "newmodel");
  edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);

  elementPose = addChild(element, "Pose", "");
  QString pose("0 0 0 0 0 0");
  parsePoseVector(pose, elementPose);

  addChild(element, "isStatic", "false");
  addChild(element, "uri", "model://" + ui->comboBox->currentText());
}

void DialogNewModel::parsePoseVector(const QString& data,
                                     QTreeWidgetItem* element)
{
  if (!data.isEmpty())
  {
    QStringList pose = data.split(" ", QString::SkipEmptyParts);
    if (pose.length() == 6)
    {
      QTreeWidgetItem* edit;
      edit = addChild(element, "X", pose.at(0).trimmed());
      edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
      edit = addChild(element, "Y", pose.at(1).trimmed());
      edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
      edit = addChild(element, "Z", pose.at(2).trimmed());
      edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
      edit = addChild(element, "Roll", pose.at(3).trimmed());
      edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
      edit = addChild(element, "Pitch", pose.at(4).trimmed());
      edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
      edit = addChild(element, "Yaw", pose.at(5).trimmed());
      edit->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
    }
  }
}
