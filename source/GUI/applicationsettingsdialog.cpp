#include "applicationsettingsdialog.h"
#include "ui_applicationsettingsdialog.h"

#include <QDir>
#include <QMessageBox>

#include "Utils/appsettings.h"
#include "Widgets/filebrowserwidget.h"

ApplicationSettingsDialog::ApplicationSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ApplicationSettingsDialog)
{
    ui->setupUi(this);

    AppSettings settings;

    // Configura o Widget para o caminho Provant ROS
    ui->provantRosLabel->setText(PROVANT_ROS_KEY);
    ui->provantROSWidget->setBrowserDialogCaption(tr("Provant ROS"));
    ui->provantROSWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->provantROSWidget->setToolTip(
                tr("Path to the directory containing the catkin workspace "
                   "source files. This directory is usually the \"src\" "
                   "directory inside your catkin workspace."));
    ui->provantROSWidget->setFilePath(settings.getProvanRosPathUnchecked());

    // Configura o Widget para o caminho de Tilt Strategies
    ui->tiltStrategiesLabel->setText(TILT_STRATEGIES_KEY);
    ui->tiltStrategiesWidget->setBrowserDialogCaption(tr("Tilt Strategies"));
    ui->tiltStrategiesWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->tiltStrategiesWidget->setToolTip(
                tr("Path to the directoy containing the compiled binaries of "
                   "the  controllers. This directory is usually inside the "
                   "catkin workspace in the \"devel/lib\" directory."));
    ui->tiltStrategiesWidget->setFilePath(
                settings.getTiltStratigiesPathUnchecked());

    // Configura o Widget para o caminho de Tilt Project
    ui->tiltProjectLabel->setText(TILT_PROJECT_KEY);
    ui->tiltProjectWidget->setBrowserDialogCaption(tr("Tilt Project"));
    ui->tiltProjectWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->tiltProjectWidget->setToolTip(
                tr("Path to the root directory of the Provant Simulator "
                   "files. This directory is usually inside the \"src/ "
                   "ProVANT-Simulator\" directory in your catkin workspace."));
    ui->tiltProjectWidget->setFilePath(settings.getTiltProjectPathUnchecked());

    // Configura o Widget para o caminho de Tilt Matlab
    ui->tiltMatlabLabel->setText(TILT_MATLAB_KEY);
    ui->tiltMatlabWidget->setBrowserDialogCaption(tr("Tilt Matlab"));
    ui->tiltMatlabWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->tiltMatlabWidget->setToolTip(
                tr("Path to directory containing the files necessary to export "
                   "the logs of the controller excecution to a Matlab file "
                   "format. Usually this directory is inside the path to the "
                   "\"TILT_PROJECT\" diretory under \"source/Structure/"
                   "Matlab\""));
    ui->tiltMatlabWidget->setFilePath(settings.getTiltMatlabPathUnchecked());

    // Configura o Widget para o caminho de Provant Database
    ui->provantDatabaseLabel->setText(PROVANT_DATABASE_KEY);
    ui->provantDatabaseWidget->setBrowserDialogCaption(tr("Provant Database"));
    ui->provantDatabaseWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->provantDatabaseWidget->setToolTip(
                tr("Path to the directory containing the database ROS package "
                   "provided with the Provant Simulator. Usually this "
                   "directory is inside the path to the \"TILT_PROJECT\" "
                   "directory under \"source/Database/\""));
    ui->provantDatabaseWidget->setFilePath(
                settings.getProvantDatabsePathUnchecked());

    // Configura o Widget para o caminho de Gazebo Model Path
    ui->gazeboModelPathLabel->setText(GAZEBO_MODEL_PATH_KEY);
    ui->gazeboModelWidget->setBrowserDialogCaption(tr("Gazebo Model Path"));
    ui->gazeboModelWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->gazeboModelWidget->setToolTip(
                tr("Path to the directory which contains the "
                   "simulator models. This directory is usually inside the "
                   "path to \"PROVANT_DATABSE\" directory under \"models/\"."));
    ui->gazeboModelWidget->setFilePath(settings.getGazeboModelPathUncheked());

    // Configura o Widget para o caminho de DIR_ROS
    ui->dirRosLabel->setText(DIR_ROS_KEY);
    ui->dirRosWidget->setBrowserDialogCaption(tr("Dir ROS"));
    ui->dirRosWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->dirRosWidget->setToolTip(
                tr("Path to your catkin workspace."));
    ui->dirRosWidget->setFilePath(settings.getRosPathUnchecked());
}

ApplicationSettingsDialog::~ApplicationSettingsDialog()
{
    delete ui;
}

void ApplicationSettingsDialog::showErrorOnSettingMessage(const QString &key)
const
{
    QMessageBox::critical(
                nullptr,
                tr("Error when setting the %1 path").arg(key),
                tr("The path configured under the %1 option is invalid."
                   "Please provide a valid path and try again.").arg(key));
}

void ApplicationSettingsDialog::accept()
{
    AppSettings settings;

    if(!settings.setProvantRosPath(ui->provantROSWidget->filePath()))
    {
        showErrorOnSettingMessage(PROVANT_ROS_KEY);
    }
    else if(!settings.setTiltStrategiesPath(
                ui->tiltStrategiesWidget->filePath()))
    {
        showErrorOnSettingMessage(TILT_STRATEGIES_KEY);
    }
    else if(!settings.setTiltProjectPath(ui->tiltProjectWidget->filePath()))
    {
        showErrorOnSettingMessage(TILT_PROJECT_KEY);
    }
    else if(!settings.setTiltMatlabPath(ui->tiltMatlabWidget->filePath()))
    {
        showErrorOnSettingMessage(TILT_MATLAB_KEY);
    }
    else if(!settings.setProvantDatabasePath(
                ui->provantDatabaseWidget->filePath()))
    {
        showErrorOnSettingMessage(PROVANT_DATABASE_KEY);
    }
    else if(!settings.setGazeboModelPath(ui->gazeboModelWidget->filePath()))
    {
        showErrorOnSettingMessage(GAZEBO_MODEL_PATH_KEY);
    }
    else if(!settings.setRosPath(ui->dirRosWidget->filePath())) {
        showErrorOnSettingMessage(DIR_ROS_KEY);
    }
    else {
        QDialog::accept();
    }
}

void ApplicationSettingsDialog::reject()
{
    QDialog::reject();
}
