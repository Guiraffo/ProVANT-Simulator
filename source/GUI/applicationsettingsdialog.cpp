#include "applicationsettingsdialog.h"
#include "ui_applicationsettingsdialog.h"

#include <QDir>
#include <QMessageBox>

#include "Utils/appsettings.h"
#include "Widgets/filebrowserwidget.h"

/*!
 * \brief ApplicationSettingsDialog::ApplicationSettingsDialog
 * \param parent The QWidget which contains this window.
 *
 * Constructs the ApplicationSettingsDialog and initializes the widgets
 * contained in the user interface.
 *
 * The configuration of the current values of the properties and setup of the
 * FileBrowserWidgets is done here.
 *
 * The following actions should be performed for each of the configuration
 * parameters:
 * * Set the text of the label using the constants defined in appsettings.h;
 * * Set the caption text of the file browser widget;
 * * Set the type of the file browser widget;
 * * Set the tool tip of the file browser widget explaining what the parameter
 * is and a good starting value for this paramter;
 * * Set the current value of the parameter;
 */
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
    ui->provantROSWidget->setFilePath(settings.getProvantRosPathUnchecked());

    // Configura o Widget para o caminho de Tilt Strategies
    ui->tiltStrategiesLabel->setText(TILT_STRATEGIES_KEY);
    ui->tiltStrategiesWidget->setBrowserDialogCaption(tr("Tilt Strategies"));
    ui->tiltStrategiesWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->tiltStrategiesWidget->setToolTip(
                tr("Path to the directoy containing the compiled binaries of "
                   "the  controllers. This directory is usually inside the "
                   "catkin workspace in the \"devel/lib\" directory."));
    ui->tiltStrategiesWidget->setFilePath(
                settings.getTiltStrategiesPathUnchecked());

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
                settings.getProvantDatabasePathUnchecked());

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
    ui->dirRosWidget->setFilePath(settings.getCatkinWorkspacePathUncheked());

    // Configura o Widget para o caminho de CONTROL_STRATEGIES_PATH
    ui->controlStrategiesSourceLabel->setText(CONTROL_STRATEGIES_SOURCE_KEY);
    ui->controlStrategiesSourceWidget->setBrowserDialogCaption(
                tr("Control Strategies Source"));
    ui->controlStrategiesSourceWidget->setBrowserType(
                FileBrowserWidget::Directory);
    ui->controlStrategiesSourceWidget->setToolTip(
                tr("Path to the directory which contains the source files to "
                   "all of the control strategies that can be used in the "
                   "simulator. This directory is usually inside the path to "
                   "\"%1\" under \"/source/Structure/control_strategies/\".")
                .arg(TILT_PROJECT_KEY));
    ui->controlStrategiesSourceWidget->setFilePath(
                settings.getControlStrategiesPathUnchecked());

    //Configure o Widget para o caminho de ROS_PATH
    ui->rosPathLabel->setText(ROS_PATH_KEY);
    ui->rosPathWidget->setBrowserDialogCaption(tr("ROS Installation path"));
    ui->rosPathWidget->setBrowserType(FileBrowserWidget::Directory);
    ui->rosPathWidget->setToolTip(
                tr("Path to the directory which contains the installation "
                   "of ROS. This directory is usually inside /opt/ros/"
                   "<ros_version>/"));
    ui->rosPathWidget->setFilePath(settings.getRosPathUnchecked());

    //Configure o Widget para o caminho de ROS_VERSION
    ui->rosVersionLabel->setText(ROS_VERSION_KEY);
    ui->rosVersionWidget->addItem("melodic");
    ui->rosVersionWidget->addItem("kinetic");
    ui->rosVersionWidget->setCurrentText(settings.getRosVersionUnchecked());
    ui->rosVersionWidget->setToolTip(
                tr("The ROS version used to run the simulator."));
}

/**
 * @brief ApplicationSettingsDialog::~ApplicationSettingsDialog
 *
 * Deletes the objects contained in this object user interface.
 */
ApplicationSettingsDialog::~ApplicationSettingsDialog()
{
    delete ui;
}

/**
 * @brief ApplicationSettingsDialog::showErrorOnSettingMessage
 *
 * Show a message informing the user that the value contained in a
 * configuration parameter is invalid.
 *
 * @param key Text informing in which parameter the error ocurred.
 *
 * Shows a QMessageBox of the type critical informing the user that one
 * parameter configuration is invalid.
 *
 * The text is followed by a brief explanation on how to fix this error and
 * requesting that the user do the actions informed and try again.
 */
void ApplicationSettingsDialog::showErrorOnSettingMessage(const QString &key)
const
{
    QMessageBox::critical(
                nullptr,
                tr("Error when setting the %1 path").arg(key),
                tr("The path configured under the %1 option is invalid."
                   "Please provide a valid path and try again.").arg(key));
}

/**
 * @brief ApplicationSettingsDialog::accept
 * Method called when the user finishes modifying the parameters and cliks
 * on the Ok button in the user interface.
 *
 * This function tries to update the value of the parameters using the
 * apropriate set mehtods in the AppSettings class.
 *
 * If an error is detected in any parameter an error message is shown to the
 * user informing about the error ocurrence and an orientation on how to fix
 * the error, and no other action is performed.
 *
 * If all values are correctly configured, they are updated and the window
 * is closed by calling the accept method of the base class.
 */
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
    else if(!settings.setCatkinWorkspacePath(ui->dirRosWidget->filePath()))
    {
        showErrorOnSettingMessage(DIR_ROS_KEY);
    }
    else if(!settings.setControlStrategiesPath(
                ui->controlStrategiesSourceWidget->filePath()))
    {
        showErrorOnSettingMessage(CONTROL_STRATEGIES_SOURCE_KEY);
    }
    else if(!settings.setRosPath(ui->rosPathWidget->filePath()))
    {
        showErrorOnSettingMessage(ROS_PATH_KEY);
    }
    else if(!settings.setRosVersion(ui->rosVersionWidget->currentText()))
    {
        showErrorOnSettingMessage(ROS_VERSION_KEY);
    }
    else {
        QDialog::accept();
    }
}

/**
 * @brief ApplicationSettingsDialog::reject
 * The action performed when the user closes the dialog without accepting the
 * changes made.
 *
 * This function makes no action other than calling the reject method of the
 * base class.
 */
void ApplicationSettingsDialog::reject()
{
    QDialog::reject();
}
