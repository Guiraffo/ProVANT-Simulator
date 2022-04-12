#include "modelsetupdialog.h"
#include "ui_modelsetupdialog.h"

#include <QtGlobal>
#include <QMessageBox>
#include <QDesktopServices>
#include <QtSerialPort/QtSerialPort>
#include <QByteArray>

#include "Business/treeitens.h"
#include "dialognewcontroller.h"
#include "Utils/appsettings.h"
#include "ProcessOutput/processoutputwindow.h"

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
ModelSetupDialog::ModelSetupDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::ModelSetupDialog)
{
  ui->setupUi(this);
  ui->treeWidget->setColumnCount(2);
  ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);
  ui->tabWidget->setCurrentIndex(0);
  // Fill the list of USB ports
  on_refreshUSB_clicked();
  fillBaudrateSelector();
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
void ModelSetupDialog::setModel(QString modelFile, QString controllerFile)
{
  // Get model and the controller from the file.
  _model.getFirst(modelFile, ui->treeWidget);
  _controller.open(controllerFile, ui->sensorsListWidget,
                   ui->actuatorsListWidget);

  // List all availabe controllers
  updateAvailableControllers();

  // Updates the other options
  ui->SampleEdit->setText(_controller.config->getSampleTime());
  ui->ErrorEdit->setText(_controller.config->getErrorLogFilename());
  ui->ActuatorEdit->setText(_controller.config->getOutLogFilename());
  ui->SensorEdit->setText(_controller.config->getInLog());
  ui->ReferenceEdit->setText(_controller.config->getRefLogFilename());
  if (_controller.config->getSimulationDuration() == 0)
  {
    ui->limitedDurationCheckBox->setChecked(false);
    ui->simulationDuration->setEnabled(false);
    ui->shutdownWhenFinishedCheckBox->setEnabled(false);
  }
  else
  {
    ui->limitedDurationCheckBox->setChecked(true);
    ui->simulationDuration->setEnabled(true);
    ui->shutdownWhenFinishedCheckBox->setEnabled(true);
    ui->simulationDuration->setTime(QTime::fromMSecsSinceStartOfDay(
        _controller.config->getSimulationDuration() * _stepTimeUs / 1000));
  }

  ui->shutdownWhenFinishedCheckBox->setChecked(
      _controller.config->getShutdownWhenFinished());

  ui->startPausedCheckbox->setChecked(_controller.config->getStartPaused());

  // Set HIL options
  const int baudrateIndex =
      ui->selectBaudRate->findData(_controller.config->getBaudRate());
  if (baudrateIndex != -1)
  {
    ui->selectBaudRate->setCurrentIndex(baudrateIndex);
  }

  const QString port1 = _controller.config->getUsart1();
  if (!port1.isEmpty())
  {
    const int portIndex = ui->selectUSB1->findData(port1);
    if (portIndex != -1)
      ui->selectUSB1->setCurrentIndex(portIndex);
  }

  const QString port2 = _controller.config->getUsart2();
  if (!port2.isEmpty())
  {
    const int portIndex = ui->selectUSB2->findData(port2);
    if (portIndex != -1)
      ui->selectUSB2->setCurrentIndex(portIndex);
  }

  ui->hilCheckBoxSync->setChecked(_controller.config->getHilFlagSynchronous());
  ui->hilCheckBoxAsync->setChecked(
      _controller.config->getHilFlagAsynchronous());

  ui->tabWidget->setCurrentIndex(0);
}

/**
 * @brief ModelSetupDialog::setStepTime
 * Set the simulation step time. Used to update the step time interval.
 * This will be used to calculate the number of steps that a simulation has.
 * @param us Simulation step time in micro seconds.
 */
void ModelSetupDialog::setStepTime(uint64_t us)
{
  _stepTimeUs = us;
}


/**
 * @brief ModelSetupDialog::hil
 * @return The state of the hardware on the loop flag.
 */
bool ModelSetupDialog::hilAsync() const
{
  return _hilAsync;
}
bool ModelSetupDialog::hilSync() const
{
  return _hilSync;
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
  if (newControllerWindow.exec() == QDialog::Accepted)
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

  if (ui->controllerComboBox->count() == 0 ||
      ui->controllerComboBox->currentText().isEmpty())

  {
    qWarning("Trying to compile a control strategy, but no control "
             "strategy is selected.");
  }
  else if(_hilSync)
  {
      QProcess* compileProcess = new QProcess(this);
      // Sets the full path of catkin_make application
      compileProcess->setProgram(
          QDir::cleanPath(settings.getRosPath() + QDir::separator() + "bin" +
                          QDir::separator() + "catkin_make"));
      QStringList args;
      // Sets the list of arguments to the program
      args << "--pkg" << "vant2_lqr";
      args << "--directory" << settings.getCatkinWorkspacePath();
      compileProcess->setArguments(args);
      // Sets the working directory as the user catkin workspace
      compileProcess->setWorkingDirectory(settings.getCatkinWorkspacePath());
      // Sets the environment variables of the new process
      compileProcess->setProcessEnvironment(settings.getEnvironmentVariables());

      // Create a window, start the process and show it to the user
      ProcessOutputWindow* processOutput =
          new ProcessOutputWindow(compileProcess, this);
      processOutput->setProcessName(
          QString("Compiling the %1 controller")
              .arg(ui->controllerComboBox->currentText()));
      processOutput->show();
      processOutput->start();
  }
  else if (!rosPath.isEmpty())
  {
    QProcess* compileProcess = new QProcess(this);
    // Sets the full path of catkin_make application
    compileProcess->setProgram(
        QDir::cleanPath(settings.getRosPath() + QDir::separator() + "bin" +
                        QDir::separator() + "catkin_make"));
    QStringList args;
    // Sets the list of arguments to the program
    args << "--pkg" << ui->controllerComboBox->currentText();
    args << "--directory" << settings.getCatkinWorkspacePath();
    compileProcess->setArguments(args);
    // Sets the working directory as the user catkin workspace
    compileProcess->setWorkingDirectory(settings.getCatkinWorkspacePath());
    // Sets the environment variables of the new process
    compileProcess->setProcessEnvironment(settings.getEnvironmentVariables());

    // Create a window, start the process and show it to the user
    ProcessOutputWindow* processOutput =
        new ProcessOutputWindow(compileProcess, this);
    processOutput->setProcessName(
        QString("Compiling the %1 controller")
            .arg(ui->controllerComboBox->currentText()));
    processOutput->show();
    processOutput->start();
  }
  else
  {
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

  if (ui->controllerComboBox->count() == 0 ||
      ui->controllerComboBox->currentText().isEmpty())
  {
    qWarning("Trying to open the controller folder, but no controller "
             "is selected.");
  }
  else if (!path.isEmpty())
  {
    QString selectedControlStrategyPath = QDir::cleanPath(
        path + QDir::separator() + ui->controllerComboBox->currentText());
    QUrl destUrl(selectedControlStrategyPath);
    if (!QDesktopServices::openUrl(destUrl))
    {
      qCritical("An error ocurred when trying to open the file browser. "
                "The QDesktopServices returned an error.");
    }
  }
  else
  {
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
  item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
}

/**
 * @brief ModelSetupDialog::on_removeSensorButton_clicked
 * Removes the currently selected sensor from the list.
 */
void ModelSetupDialog::on_removeSensorButton_clicked()
{
  // Removes and manually deletes the item from the list
  QListWidgetItem* item =
      ui->sensorsListWidget->takeItem(ui->sensorsListWidget->currentRow());
  if (item != nullptr)
    delete item;
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
  item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
}

/**
 * @brief ModelSetupDialog::on_removeActuatorButton_clicked
 * Removes the currently selected actuator from the list.
 */
void ModelSetupDialog::on_removeActuatorButton_clicked()
{
  // Removes the item from the list and then manually frees the alocated
  // memory.
  QListWidgetItem* item =
      ui->actuatorsListWidget->takeItem(ui->actuatorsListWidget->currentRow());
  if (item != nullptr)
    delete item;
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
  _controller.config->setLog(ui->ErrorEdit->text(), ui->ReferenceEdit->text(),
                             ui->ActuatorEdit->text(), ui->SensorEdit->text());

  // Update the sensors
  _controller.config->clearSensorsAndActuators();
  for (int i = 0; i < ui->sensorsListWidget->count(); ++i)
  {
    _controller.config->addSensor(ui->sensorsListWidget->item(i)->text());
  }
  // Update the actuators
  for (int i = 0; i < ui->actuatorsListWidget->count(); ++i)
  {
    _controller.config->addActuator(ui->actuatorsListWidget->item(i)->text());
  }
  // Set the simulation duration
  if (ui->limitedDurationCheckBox->isChecked())
  {
    _controller.config->setSimulationDuration(calcSimulationSteps());
  }
  else
  {
    _controller.config->setSimulationDuration(0);
  }
  // Set the shutdown when finished value
  _controller.config->setShutdownWhenFinished(
      ui->shutdownWhenFinishedCheckBox->isChecked());
  // Set the start paused value
  _controller.config->setStartPaused(ui->startPausedCheckbox->isChecked());

  // Set the hil options
  _controller.config->setHilFlagSynchronous(ui->hilCheckBoxSync->isChecked());
  _controller.config->setHilFlagAsynchronous(ui->hilCheckBoxAsync->isChecked());

  {
    bool ok;
    const int baudrate = ui->selectBaudRate->currentData().toInt(&ok);
    if (ok)
    {
      _controller.config->setBaudRate(baudrate);
    }
    else
    {
      _controller.config->setBaudRate(0);
    }
  }

  if (ui->selectUSB1->count() > 0 && ui->selectUSB1->currentIndex() != 0)
  {
    _controller.config->setUsart1(ui->selectUSB1->currentData().toString());
  }
  else
  {
    _controller.config->setUsart1("");
  }

  if (ui->selectUSB2->count() > 0 && ui->selectUSB2->currentIndex() != 0)
  {
    _controller.config->setUsart2(ui->selectUSB2->currentData().toString());
  }
  else
  {
    _controller.config->setUsart2("");
  }

  // Finishes the process and saves the file.
  _controller.config->writeFile();
}

/**
 * @brief ModelSetupDialog::on_hilCheckBox_clicked Slot to update the value of
 * the hardware on the loop flag.
 * @param checked Status of the hardware on the loop checkbox.
 */
void ModelSetupDialog::on_hilCheckBoxAsync_clicked(bool checked)
{
  setHilAsync(checked);
}

void ModelSetupDialog::on_hilCheckBoxSync_clicked(bool checked)
{
  setHilSync(checked);
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

  if (controlStrategiesDir.exists())
  {
    // Reads all available control strategies
    QFileInfoList files =
        controlStrategiesDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

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
      if (currentController == selectedController)
      {
        selectedControllerIndex = currentControllerIndex;
      }
      currentControllerIndex++;
    }

    // If the selected controller is found, updated the combobox.
    if (selectedControllerIndex != -1)
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
      QMessageBox::warning(nullptr, tr("Controller Not Found"),
                           tr("The selected controller (%1) for the current "
                              "model "
                              "was not found among the available control "
                              "strategies, please select a new controller or "
                              "include the desired controller source code in "
                              "the "
                              "correct location.")
                               .arg(selectedController));
    }
  }
  else
  {
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
void ModelSetupDialog::setHilAsync(bool enabled)
{
  if (enabled != _hilAsync)
  {
    _hilAsync = enabled;
    _controller.config->setHilFlagAsynchronous(enabled);
    _controller.config->setHilFlagSynchronous(!enabled);

    if (enabled)
      ui->hilCheckBoxSync->setChecked(false);
    ui->hilCheckBoxSync->setDisabled(enabled);

    ui->SampleEdit->setEnabled(!enabled);
    ui->ErrorEdit->setEnabled(!enabled);
    ui->ActuatorEdit->setEnabled(!enabled);
    ui->SensorEdit->setEnabled(!enabled);
    ui->ReferenceEdit->setEnabled(!enabled);
    ui->newControllerButton->setEnabled(!enabled);
    ui->openControllerButton->setEnabled(!enabled);
    ui->compileControllerButton->setEnabled(!enabled);
    ui->controllerComboBox->setEnabled(!enabled);

    emit hilFlagChanged(enabled);
  }
}

void ModelSetupDialog::setHilSync(bool enabled)
{
  if (enabled != _hilSync)
  {
    _hilSync = enabled;
    _controller.config->setHilFlagAsynchronous(!enabled);
    _controller.config->setHilFlagSynchronous(enabled);

    if (enabled)
      ui->hilCheckBoxAsync->setChecked(false);
    ui->hilCheckBoxAsync->setDisabled(enabled);

    ui->SampleEdit->setEnabled(!enabled);
    ui->ErrorEdit->setEnabled(!enabled);
    ui->ActuatorEdit->setEnabled(!enabled);
    ui->SensorEdit->setEnabled(!enabled);
    ui->ReferenceEdit->setEnabled(!enabled);
    ui->newControllerButton->setEnabled(!enabled);
    ui->openControllerButton->setEnabled(!enabled);
    ui->controllerComboBox->setEnabled(!enabled);
  }
}

/**
 * @brief ModelSetupDialog::on_compileHilButton_clicked
 *
 * @todo Why is this button necessary?
 * The code used in the ProVANT Simulator for communication with the embedded
 * controller should not be altered by an user in regular use cases, so why do
 * we need to offer a compile button?
 * @todo Fix the selected control strategy that is currently being compiled, the
 * vant2_lqr has nothing to do with HIL.
 * @todo Fix the error messages. The error messages are logged, but the log
 * messages are too generic, and not shown in the UI to the user. The correct
 * behavior is to use instances of the QMessageBox class to show messages to
 * user. A good error message should indicate 1) What was the error. 2) Where
 * and when the error ocurred. 3) If known, what was the error cause. 4) How to
 * fix the error. For example, the message "Please select the USB ports before
 * compiling.", it indicates the error (the USB ports were not selected before
 * compilation), it indicates where and when the error ocurred (the error
 * ocurred after clicking the compile button), it indicates the error cause (the
 * USB ports should be selected before compilation), and how to fix it (the user
 * should select the USB ports).
 */
void ModelSetupDialog::on_compileHilButton_clicked(){
    AppSettings settings;

    if ((ui->selectUSB1->count() == 0 ||
        ui->selectUSB1->currentText().isEmpty()) &
       (ui->selectUSB2->count() == 0 ||
        ui->selectUSB2->currentText().isEmpty()))
    {
      qWarning("Select the USB ports");
    }
    else if(!(ui->selectUSB1->count() == 0 ||
        ui->selectUSB1->currentText().isEmpty()) &
       !(ui->selectUSB2->count() == 0 ||
         ui->selectUSB2->currentText().isEmpty()) & !(_hilAsync || _hilSync))
    {
      qWarning("Select the HIL Mode");
    }
    else if(ui->selectBaudRate->currentText().isEmpty())
    {
      qWarning("Select Baud Rate");
    }
    else if(_hilSync)
    {
        QProcess* compileProcess = new QProcess(this);
        // Sets the full path of catkin_make application
        compileProcess->setProgram(
            QDir::cleanPath(settings.getRosPath() + QDir::separator() + "bin" +
                            QDir::separator() + "catkin_make"));
        QStringList args;
        // Sets the list of arguments to the program
        args << "--pkg" << "vant2_lqr";
        args << "--directory" << settings.getCatkinWorkspacePath();
        compileProcess->setArguments(args);
        // Sets the working directory as the user catkin workspace
        compileProcess->setWorkingDirectory(settings.getCatkinWorkspacePath());
        // Sets the environment variables of the new process
        compileProcess->setProcessEnvironment(settings.getEnvironmentVariables());

        // Create a window, start the process and show it to the user
        ProcessOutputWindow* processOutput =
            new ProcessOutputWindow(compileProcess, this);
        processOutput->setProcessName(
            QString("Compiling the %1 controller")
                .arg(ui->controllerComboBox->currentText()));
        processOutput->show();
        processOutput->start();

        QString usart1 = ui->selectUSB1->currentText();
        QStringList usart1_list = usart1.split(QLatin1Char(' '));;
        usart1 = usart1_list[0];

        QString usart2 = ui->selectUSB2->currentText();
        QStringList usart2_list = usart2.split(QLatin1Char(' '));
        usart2 = usart2_list[0];

        if(usart1 == usart2)
        {
            qWarning("Choice diferent USB Ports");
        }
        else {
            _controller.config->setUsart1(usart1);
            _controller.config->setUsart2(usart2);
        }
        _controller.config->setBaudRate(ui->selectBaudRate->currentText().toInt());
    }
    else if(_hilAsync)
    {
        QString usart1 = ui->selectUSB1->currentText();
        QStringList usart1_list = usart1.split(QLatin1Char(' '));;
        usart1 = usart1_list[0];

        QString usart2 = ui->selectUSB2->currentText();
        QStringList usart2_list = usart2.split(QLatin1Char(' '));
        usart2 = usart2_list[0];

        if(usart1 == usart2)
        {
            qWarning("Choice diferent USB Ports");
        }
        else {
            _controller.config->setUsart1(usart1);
            _controller.config->setUsart2(usart2);
        }
    }
    else{
        qWarning("An Error occurred");
    }
    _controller.config->setBaudRate(ui->selectBaudRate->currentText().toInt());
}

void ModelSetupDialog::on_refreshUSB_clicked()
{
  ui->selectUSB1->setEnabled(false);
  ui->selectUSB2->setEnabled(false);

  const QString currentUSB1 = ui->selectUSB1->currentIndex() == 0 ?
                                  "" :
                                  ui->selectUSB1->currentData().toString();
  const QString currentUSB2 = ui->selectUSB2->currentIndex() == 0 ?
                                  "" :
                                  ui->selectUSB2->currentData().toString();

  ui->selectUSB1->clear();
  ui->selectUSB2->clear();

  foreach (const QSerialPortInfo& info, QSerialPortInfo::availablePorts())
  {
    const QString portText =
        info.manufacturer().isNull() ?
            info.systemLocation() :
            info.systemLocation() + " - " + info.manufacturer();
    ui->selectUSB1->addItem(portText, info.systemLocation());
    ui->selectUSB2->addItem(portText, info.systemLocation());
  }

  const QString selectPortText = tr("Select port");

  if (ui->selectUSB1->count() > 0)
  {
    ui->selectUSB1->insertItem(0, selectPortText);
    ui->selectUSB1->setCurrentIndex(0);
    ui->selectUSB1->setEnabled(true);
  }

  if (ui->selectUSB2->count() > 0)
  {
    ui->selectUSB2->insertItem(0, selectPortText);
    ui->selectUSB2->setCurrentIndex(0);
    ui->selectUSB2->setEnabled(true);
  }

  // Check the previously selected ports are avilable, and if so, select them
  if (!currentUSB1.isEmpty())
  {
    const int index = ui->selectUSB1->findData(currentUSB1);
    if (index != -1)
    {
      ui->selectUSB1->setCurrentIndex(index);
    }
  }

  if (!currentUSB2.isEmpty())
  {
    const int index = ui->selectUSB2->findData(currentUSB2);
    if (index != -1)
    {
      ui->selectUSB2->setCurrentIndex(index);
    }
  }
}

void ModelSetupDialog::on_selectUSB1_currentIndexChanged(int index){
  Q_UNUSED(index);
}

void ModelSetupDialog::on_selectUSB2_currentIndexChanged(int index)
{
  Q_UNUSED(index);
}

void ModelSetupDialog::on_turbulanceCheckBox_clicked()
{
  _controller.config->setTurbulanceModel(ui->turbulanceComboBox->currentText());
}

void ModelSetupDialog::on_treeWidget_itemDoubleClicked(QTreeWidgetItem* item,
                                                       int column)
{
  if (column == 1)
  {
    if (item->flags() & Qt::ItemIsEditable)
      ui->treeWidget->editItem(item, column);
  }
}

uint64_t ModelSetupDialog::calcSimulationSteps() const
{
  uint64_t duration = ui->simulationDuration->time().msecsSinceStartOfDay();
  duration *= 1000;

  if (_stepTimeUs != 0)
  {
    return static_cast<uint64_t>(ceil(duration / _stepTimeUs));
  }
  return 0;
}

void ModelSetupDialog::fillBaudrateSelector()
{
  ui->selectBaudRate->clear();

  foreach (qint32 baudrate, QSerialPortInfo::standardBaudRates())
  {
    ui->selectBaudRate->addItem(QString::number(baudrate) + " bps", baudrate);
  }
}
