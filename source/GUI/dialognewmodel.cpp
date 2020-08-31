#include "dialognewmodel.h"
#include "ui_dialognewmodel.h"

#include "Business/treeitens.h"
#include "Utils/appsettings.h"

#include <QDebug>
#include <QDir>
#include <QMessageBox>
#include <QString>

DialogNewModel::DialogNewModel(Ui::MainWindow* mainWindow, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialognewmodel),
    parentUi(mainWindow)
{
    ui->setupUi(this);
    findModels();
}

DialogNewModel::~DialogNewModel()
{
    delete ui;
}

void DialogNewModel::findModels()
{
    AppSettings settings;
    QString gazeboModelPath = settings.getGazeboModelPath();
    QDir gazeboModelsDir(gazeboModelPath);
    if(gazeboModelsDir.exists()) {
        QFileInfoList files = gazeboModelsDir.entryInfoList(
                    QDir::Dirs | QDir::NoDotAndDotDot | QDir::Readable);
        foreach (QFileInfo file, files)
        {
            QDir modelDir(file.absoluteFilePath());
            // Check that the model has a config.xml file
            QFileInfo configFile(
                        modelDir.absoluteFilePath(
                            QDir::cleanPath(QString("config")
                                            + QDir::separator()
                                            + QString("config.xml"))));
            if(!configFile.exists()) {
                // Skip models that don't have a config file
                continue;
            }
            // Check that the model has a SDF file
            QFileInfo sdfFile(
                        modelDir.absoluteFilePath(
                            QDir::cleanPath(QString("robot")
                                            + QDir::separator()
                                            + QString("model.sdf")))
                        );
            if(!sdfFile.exists()) {
                // Skip models that don't have a SDF file
                continue;
            }

            ui->comboBox->addItem(modelDir.dirName());
        }
    }
    else {
        QMessageBox::critical(this,
                              tr("Error"),
                              tr("Error while trying to open the gazebo model "
                                 "path direcotry. Please make sure the "
                                 "GAZEBO_MODEL_PATH environment variable is "
                                 "correctly set. To check this value access "
                                 "the Tools -> Options menu in the main "
                                 "window."));
        qCritical("Error while trying to open the GAZEBO_MODEL_PATH with "
                  "value %s.",
                  qUtf8Printable(gazeboModelPath));
    }
}

void DialogNewModel::on_buttonBox_accepted()
{
    AppSettings settings;
    QString gazeboModelpath = settings.getGazeboModelPath();

    // Add the new model to the tree widget item of the world configuration
    QTreeWidgetItem *element, *elementPose, *edit;

    element = addRoot("Include", "", parentUi->treeWidget);
    edit = addChild(element, "name", "newmodel");
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);

    elementPose = addChild(element, "Pose", "");
    QString pose("0 0 0 0 0 0");
    parsePoseVector(pose, elementPose);

    addChild(element, "isStatic", "false");
    addChild(element, "uri", "model://"+ui->comboBox->currentText());
}

void DialogNewModel::parsePoseVector(const QString &data,
                                     QTreeWidgetItem *element)
{
    if(!data.isEmpty())
    {
        QStringList pose = data.split(" ", QString::SkipEmptyParts);
        if(pose.length() == 6) {
            QTreeWidgetItem* edit;
            edit = addChild(element, "X", pose.at(0).trimmed());
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
            edit = addChild(element, "Y", pose.at(1).trimmed());
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
            edit = addChild(element, "Z", pose.at(2).trimmed());
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
            edit = addChild(element, "Roll", pose.at(3).trimmed());
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
            edit = addChild(element, "Pitch", pose.at(4).trimmed());
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
            edit = addChild(element, "Yaw", pose.at(5).trimmed());
            edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        }
    }
}
