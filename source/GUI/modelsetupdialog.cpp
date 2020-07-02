#include "modelsetupdialog.h"
#include "ui_modelsetupdialog.h"

#include <QtGlobal>
#include <QMessageBox>
#include <QDesktopServices>

#include"Business/treeitens.h"
#include "dialognewcontroller.h"
#include "dialogopencontroller.h"
#include "Utils/appsettings.h"
#include "processoutputwindow.h"

/**
 * @brief ModelSetupDialog::ModelSetupDialog
 * Initializes a ModelSetupDialog object and constructs the user interface.
 * @param parent
 *
 * After performing t he necessary initialization of the class members and
 * of the base class constrcutor, this class sets the options to the treeWidget
 * used to display model configurations, making it two columns wide and
 * setting the edit triggers.
 */
ModelSetupDialog::ModelSetupDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelSetupDialog)
{
    ui->setupUi(this);
    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);
}

/**
 * @brief ModelSetupDialog::~ModelSetupDialog
 * Deletes the objects contained in the user interface.
 */
ModelSetupDialog::~ModelSetupDialog()
{
    delete ui;
}

/**
 * @brief ModelSetupDialog::setModel
 * @param modelFile The file containing the model.
 * @param controllerFile The file containing the controller.
 *
 * Sets the model displayed on the window and displays all the options
 * for configuration of the model.
 */
void ModelSetupDialog::setModel(QString modelFile,
                                QString controllerFile)
{
    // Get model and the controller from the file.
    _model.getFirst(modelFile.toStdString(), ui->treeWidget);
    _controller.open(controllerFile,
                   ui->sensorsListWidget,
                   ui->actuatorsListWidget);

    // List all availabe controllers
    updateAvailableControllers();

    // Updates the other options
    ui->SampleEdit->setText(_controller.config->getSampleTime());
    ui->ErrorEdit->setText(_controller.config->getErrorLogFilename());
    ui->ActuatorEdit->setText(_controller.config->getOutLogFilename());
    ui->SensorEdit->setText(_controller.config->getInLog());
    ui->ReferenceEdit->setText(_controller.config->getRefLogFilename());
}

/**
 * @brief ModelSetupDialog::hil
 * @return The state of the hardware on the loop flag.
 */
bool ModelSetupDialog::hil() const
{
    return _hil;
}

/**
 * @brief ModelSetupDialog::on_newControllerButton_clicked
 * Slot called when the new controller button is clicked.
 *
 * Opens a dialog box for the user to insert the new controller name and if
 * the action is accepted by the user, the dialog box listing the available
 * controllers is updated.
 */
void ModelSetupDialog::on_newControllerButton_clicked()
{
    /*
     * Opens a new window to allow the user to inform the name of the new
     * control strategy.
     */
    DialogNewController newControllerWindow;
    if(newControllerWindow.exec() == QDialog::Accepted)
    {
        // Updates the available control strategies in the combobox
        updateAvailableControllers();
    }
}

/*!
 * \brief ModelSetupDialog::on_compileControllerButton_clicked
 * Slot called when the compile controller button is clicked.
 *
 * Checks if a controller is selectd, if selected start a catkin_make process
 * to compile the package of the selected controller.
 *
 * @author Júnio Eduardo de Morais Aquino
 */
void ModelSetupDialog::on_compileControllerButton_clicked()
{
    // Get catkin_ws folder path
    AppSettings settings;
    QString rosPath = settings.getCatkinWorkspacePath();

    if(ui->controllerComboBox->count() == 0 ||
            ui->controllerComboBox->currentText().isEmpty())

    {
        qWarning("Trying to compile a control strategy, but no control "
                 "strategy is selected.");
    }
    else if(!rosPath.isEmpty())
    {
        QProcess *compileProcess = new QProcess(this);
        // Sets the full path of catkin_make application
        compileProcess->setProgram(
                    QDir::cleanPath(settings.getRosPath() +
                                    QDir::separator() +
                                    "bin" +
                                    QDir::separator() +
                                    "catkin_make"));
        QStringList args;
        // Sets the list of arguments to the program
        args << "--pkg" << ui->controllerComboBox->currentText();
        args << "--directory" << settings.getCatkinWorkspacePath();
        compileProcess->setArguments(args);
        // Sets the working directory as the user catkin workspace
        compileProcess->setWorkingDirectory(settings.getCatkinWorkspacePath());
        // Sets the environment variables of the new process
        compileProcess->setProcessEnvironment(
                    settings.getEnvironmentVariables());

        // Create a window, start the process and show it to the user
        ProcessOutputWindow *processOutput = new ProcessOutputWindow(
                    compileProcess, this);
        processOutput->setProcessName(
                    QString("Compiling the %1 controller")
                    .arg(ui->controllerComboBox->currentText()));
        processOutput->show();
        processOutput->start();
    }
    else {
        qCritical("An error ocurred when tyring to compile the selected "
                  "control strategy (%s). The path to ROS is incorrectly "
                  "set.",
                  qUtf8Printable(rosPath));
    }
}

/**
 * @brief ModelSetupDialog::on_openControllerButton_clicked
 * Opens a file browser in the folder of the currently selected control
 * strategy.
 */
void ModelSetupDialog::on_openControllerButton_clicked()
{
    AppSettings settings;
    QString path = settings.getControlStrategiesPath();

    if(ui->controllerComboBox->count() == 0 ||
            ui->controllerComboBox->currentText().isEmpty())
    {
        qWarning("Trying to open the controller folder, but no controller "
                 "is selected.");
    }
    else if(!path.isEmpty())
    {
        QString selectedControlStrategyPath = QDir::cleanPath(
                    path +
                    QDir::separator() +
                    ui->controllerComboBox->currentText());
        QUrl destUrl(selectedControlStrategyPath);
        if(!QDesktopServices::openUrl(destUrl))
        {
            qCritical("An error ocurred when trying to open the file browser. "
                      "The QDesktopServices returned an error.");
        }
    }
    else {
        qCritical("An error ocurred when trying to open the file browser. "
                  "The necessary setting is incorrectly configured.");
    }
}

/**
 * @brief ModelSetupDialog::on_addSensorButton_clicked
 * Adds a new sensor to the list.
 */
void ModelSetupDialog::on_addSensorButton_clicked()
{
    // Creates a new list item and adds it to the lsit
    QListWidgetItem* item = new QListWidgetItem(QString(tr("New Sensor")));
    ui->sensorsListWidget->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

/**
 * @brief ModelSetupDialog::on_removeSensorButton_clicked
 * Removes the currently selected sensor from the list.
 */
void ModelSetupDialog::on_removeSensorButton_clicked()
{
    // Removes and manually deletes the item from the list
    QListWidgetItem *item = ui->sensorsListWidget->takeItem(
                ui->sensorsListWidget->currentRow());
    if(item != nullptr) delete item;
}

/**
 * @brief ModelSetupDialog::on_addActuatorButton_clicked
 * Adds a new actuator to the list.
 */
void ModelSetupDialog::on_addActuatorButton_clicked()
{
    // Creates a new object and adds it to the list.
    QListWidgetItem* item = new QListWidgetItem(QString(tr("New Actuator")));
    ui->actuatorsListWidget->addItem(item);
    item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
}

/**
 * @brief ModelSetupDialog::on_removeActuatorButton_clicked
 * Removes the currently selected actuator from the list.
 */
void ModelSetupDialog::on_removeActuatorButton_clicked()
{
    // Removes the item from the list and then manually frees the alocated
    // memory.
    QListWidgetItem *item = ui->actuatorsListWidget->takeItem(
                ui->actuatorsListWidget->currentRow());
    if (item != nullptr) delete item;
}

/**
 * @brief ModelSetupDialog::saveConfig Save the alterations made to the model
 * to the appropriate file.
 */
void ModelSetupDialog::saveConfig()
{
    // Update the control strategy library
    _controller.config->setStrategy(
                "lib" + ui->controllerComboBox->currentText() + ".so");
    // Update the sample time
    _controller.config->setSampleTime(ui->SampleEdit->text());
    // Update the files that store the results of the simulation
    _controller.config->setLog(ui->ErrorEdit->text(),
                              ui->ReferenceEdit->text(),
                              ui->ActuatorEdit->text(),
                              ui->SensorEdit->text());
    // Update the sensors
    _controller.config->clearSensorsAndActuators();
    for(int i = 0; i < ui->sensorsListWidget->count(); ++i)
    {
        _controller.config->addSensor(ui->sensorsListWidget->item(i)->text());
    }
    // Update the actuators
    for(int i = 0; i < ui->actuatorsListWidget->count(); ++i)
    {
        _controller.config->addActuator(
                    ui->actuatorsListWidget->item(i)->text());
    }
    // Finishes the process and saves the file.
    _controller.config->writeFile();
}

/**
 * @brief ModelSetupDialog::on_hilCheckBox_clicked Slot to update the value of
 * the hardware on the loop flag.
 * @param checked Status of the hardware on the loop checkbox.
 */
void ModelSetupDialog::on_hilCheckBox_clicked(bool checked)
{
    setHil(checked);
}

/**
 * @brief ModelSetupDialog::updateAvailableControllers
 * Checks the control_strategies folder for all the available controllers,
 * gets their directory name, and displays then in the controller selection
 * combobox.
 *
 * If no control strategy is available, the buttons for the compilation and
 * opening of the selected controller are disabled.
 *
 * Every folder contained under the control strategies configured path is
 * considered to be a valid controller, and no operation to validate this is
 * performed.
 *
 * @author Júnio Eduardo de Morais Aquino
 * @date 2019/02/22
 */
void ModelSetupDialog::updateAvailableControllers()
{
    // Clear the controllers currently displayed in the combobox
    ui->controllerComboBox->clear();

    // Get the path to folder containing the source code of the control
    // strategies

    AppSettings settings;
    QDir controlStrategiesDir(settings.getControlStrategiesPath());

    if (controlStrategiesDir.exists()) {
        // Reads all available control strategies
        QFileInfoList files = controlStrategiesDir.entryInfoList(
                    QDir::Dirs | QDir::NoDotAndDotDot);

        QString selectedController = _controller.config->getControlStrategy();

        /* Stores the index of the selected controller.
         * The value -1 indicates that the selected controller wasn't found.
         */
        int selectedControllerIndex = -1;
        int currentControllerIndex = 0;

        foreach (QFileInfo file, files)
        {
            ui->controllerComboBox->addItem(file.fileName());
            QString currentController = "lib" + file.fileName() + ".so";
            if(currentController == selectedController)
            {
                selectedControllerIndex = currentControllerIndex;
            }
            currentControllerIndex++;
        }

        // If the selected controller is found, updated the combobox.
        if(selectedControllerIndex != -1)
        {
            ui->controllerComboBox->setCurrentIndex(selectedControllerIndex);
        }
        else
        {
            /*
             * If the selected controller wasn't found, log the error and
             * inform the user.
             */
            qWarning("The controller (%s) was not "
                     "found in available control strategies.",
                     qUtf8Printable(selectedController));
            QMessageBox::warning(
                        nullptr,
                        tr("Controller Not Found"),
                        tr("The selected controller (%1) for the current model "
                           "was not found among the available control "
                           "strategies, please select a new controller or "
                           "include the desired controller source code in the "
                           "correct location.").arg(selectedController));
        }
    }
    else {
        /*
         * It is not necessary to create a QMessageBox informing the user about
         * the error, because the method used to return the parameter value
         * already does this.
         *
         * This error should only be logged.
         */
        qCritical("The configuration of a new control strategy could not "
                  "proceed bacause the %s parameter is incorrectly set.",
                  qUtf8Printable(TILT_PROJECT_KEY));
    }

    /*
     * If no controller is available, disables the compile and open controller
     * buttons.
     */
    bool enable = ui->controllerComboBox->count() != 0;
    ui->compileControllerButton->setEnabled(enable);
    ui->compileControllerButton->setEnabled(enable);
}

/**
 * @brief ModelSetupDialog::accept Function called when the Ok button in the
 * dialog window is clicked.
 *
 * Saves the current configuration and closes the window.
 *
 * @author Júnio Eduardo de Morais Aquino
 * @date 2020/02/21
 */
void ModelSetupDialog::accept()
{
    saveConfig();
    close();
}

/**
 * @brief ModelSetupDialog::setHil Sets the status of the hardware in the loop
 * flag and updates the user interface accordingly.
 * @param hil Status of the hardware on the loop flag.
 *
 * If the hardware in the loop mode is enabled, there is no need to alter any
 * of the other options on the controller user interface, so these options
 * are disabled.
 */
void ModelSetupDialog::setHil(bool hil)
{
    ui->SampleEdit->setEnabled(!hil);
    ui->ErrorEdit->setEnabled(!hil);
    ui->ActuatorEdit->setEnabled(!hil);
    ui->SensorEdit->setEnabled(!hil);
    ui->ReferenceEdit->setEnabled(!hil);
    ui->newControllerButton->setEnabled(!hil);
    ui->openControllerButton->setEnabled(!hil);
    ui->compileControllerButton->setEnabled(!hil);
    ui->controllerComboBox->setEnabled(!hil);

    if(hil != _hil) {
        _hil = hil;
        emit hilFlagChanged(hil);
    }
}
