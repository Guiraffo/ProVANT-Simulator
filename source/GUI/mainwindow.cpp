/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the MainWindow GUI class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include "mainwindow.h"

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QTreeWidgetItemIterator>

#include <fcntl.h>
#include <libusb-1.0/libusb.h>

#include "aboutdialog.h"
#include "applicationsettingsdialog.h"
#include "DataAccess/RosElements/roslaunch.h"
#include "dialognewmodel.h"
#include "jointsdialog.h"
#include "ProcessOutput/processoutputwindow.h"
#include "Utils/appsettings.h"
#include "Utils/checkandcloseprocess.h"
#include "Utils/modellist.h"
#include "modelsetupdialog.h"

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  // Insert logos
  QPixmap provantLogo(":/logos/logos/logo_provant.svg");
  int w = ui->provantLogoDisplay->width();
  int h = ui->provantLogoDisplay->height();
  ui->provantLogoDisplay->setPixmap(
      provantLogo.scaled(w, h, Qt::KeepAspectRatio));

  // Setup tree widget to display the simulation parameters
  ui->treeWidget->setColumnCount(2);
  ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);

  /*
   * Disables the relevant menus until the user creates a new simulation
   * world from a template, or opens a world.
   */
  ui->actionSave->setDisabled(true);
  ui->menuEdit->setDisabled(true);
  ui->startGazeboPushButton->setDisabled(true);
  ui->jointValuesPushButton->setDisabled(true);

  // Set window title
  setWindowTitle("ProVANT Simulator");

  /**
   * @brief settings Checks if all the relevant settings are correctly
   * configured. If an errror is detected, the default values are set and
   * a message is shown to the user informing about the detected errors.
   */
  AppSettings settings;
  if (settings.checkAllParametersSet())
  {
    settings.applyValuesToEnvironmentVariables();
  }

  // Check the ROS_PACKAGE_PATH env to ensure that the ROS configuration scripts
  // were correctly sourced
  checkROSEnvironment();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_startGazeboPushButton_clicked()
{
  // Get a list with all the models available under gazeboModelPath
  AppSettings settings;
  QString gazeboModelPath = settings.getGazeboModelPath();
  QDir gazeboModelPathDir(gazeboModelPath);

  // Checks if gazebo model is valid, if it isn't report the error to the
  // user and stops the execution of this method.
  if (!gazeboModelPathDir.exists())
  {
    QMessageBox::critical(this, tr("Error"),
                          tr("An error ocurred we reading the gazebo model "
                             "path, "
                             "and therefore it is not possible to start the "
                             "simulation. Please fix the configure value of "
                             "the "
                             "gazebo model path and try again."));
    qCritical() << "An error ocurred when trying to start the gazebo "
                   "simulation. The path to the gazebo model path was "
                   "invalid and the startup process was halted.";
    return;
  }

  // Get a list of all the directories under gazeboModelPath
  QFileInfoList gazeboModels =
      gazeboModelPathDir.entryInfoList(QDir::NoDotAndDotDot | QDir::Dirs);

  // A string with the path of the model config file path.
  QString modelConfigFilePath;

  // Find the include values in the tree widget
  for (int i = 0; i < ui->treeWidget->topLevelItemCount(); i++)
  {
    if (ui->treeWidget->topLevelItem(i)->text(0) == "Include")
    {
      // Fins the item with the uri
      QString modelName;
      QTreeWidgetItem* currentItem = ui->treeWidget->topLevelItem(i);
      for (int i = 0; i < currentItem->childCount(); i++)
      {
        if (currentItem->child(i)->text(0).toLower() == "uri")
        {
          modelName = currentItem->child(i)->text(1).replace("model://", "");
          break;
        }
      }

      // If a model URI was not found, continues the loop to the next
      // element
      if (modelName.isEmpty())
      {
        continue;
      }

      // Loops trough the models list
      foreach (const QFileInfo& model, gazeboModels)
      {
        // Checks if the current model directory is equal to the
        // model managed by the tree widget item.
        if (model.fileName() == modelName)
        {
          // Sets the path to the model config file and stops the
          // loop
          modelConfigFilePath = QDir::cleanPath(
              gazeboModelPath + QDir::separator() + modelName +
              QDir::separator() + "config" + QDir::separator() + "config.xml");
          break;
        }
      }

      // If the model was already found, finishes the loop
      if (!modelConfigFilePath.isEmpty())
      {
        break;
      }
    }
  }

  /*
   * If the modelConfigFilePath variable is empty, its because
   */
  if (modelConfigFilePath.isEmpty())
  {
    qCritical() << "An error ocurred while trying to start the gazebo "
                   "simulation. A valid model was not found under the "
                   "gazebo model path directory for the currently selected "
                   "world file.";
    QMessageBox::critical(this, tr("Error"),
                          tr("A model was not found under the gazebo model "
                             "path "
                             "that matches any of the models specified in this "
                             "simulation world. Please check the values of the "
                             "configured models path and of the gazebo model "
                             "path "
                             "setting and try again."));
    return;
  }

  // Stores the environment variable for the new simulation process
  QProcessEnvironment gazeboProcessEnv = settings.getEnvironmentVariables();

  // Checks if the model config file path is valid
  QFileInfo configFileInfo(modelConfigFilePath);
  if (configFileInfo.exists())
  {
    // Sets the TILT_CONFIG environment variable.
    if (!qputenv("TILT_CONFIG", modelConfigFilePath.toLocal8Bit()))
    {
      qCritical() << "An error ocurred while trying to start the gazebo "
                     "simulation. It was not possible to set the "
                     "TILT_CONFIG environment variable.";
      QMessageBox::critical(this, tr("Error"),
                            tr("A fatal error ocurred, it was not possible to "
                               "set "
                               "the value of the TILT_CONFIG environment "
                               "variable. Check your user permissions and try "
                               "again."));
      gazeboProcessEnv.insert("TILT_CONFIG", modelConfigFilePath);
      return;
    }
  }
  else
  {
    qCritical() << "An error ocurred while trying to start the gazebo "
                   "simulation. The file in the specified path "
                << modelConfigFilePath << " does not exist.";
    QMessageBox::critical(this, tr("Error"),
                          tr("The file %1 does not exist, please check the "
                             "model "
                             "configuration and the value of the gazebo model "
                             "path "
                             "setting and try again."));
    return;
  }

  QString launchFile;
  /*
   * OBS: This section of the code wont be refactored now.
   * I'm leaving it as is, note that a few modifications will be necessary
   * here to finish upgrading the use of system to QProcess.
   *
   * Further modification will be nedded to add a feature allowing the user
   * to select the USB port on the GUI.
   */
  //! @todo Refactor this section.
  if (hilAsync)
  {
    // verificar se o dispositivo está conectado em /dev/ttyUSB0

    int open_result = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (open_result == -1)
    {
      qCritical() << "Port Failed to Open";
      exit(EXIT_FAILURE);
    }
    else
    {
      std::system("setserial /dev/ttyUSB0 low_latency");
    }
    launchFile = QString("hil.launch");
  }
  else
  {
    launchFile = QString("gazebo.launch");
  }

  // Check if the currently opened file is a template.
  // If it is. Saves the file.
  if (istemplate)
  {
    if (!saveAs())
    {
      qCritical() << "Error, the template file must be saved before "
                     "running the simulation.";
      QMessageBox::critical(this, tr("Error"),
                            tr("The currently selected file is a template, so "
                               "it "
                               "must be saved before running the simulation."));
      return;
    }
  }

  // Start the simulation.
  // Saves the modifications in  the simulation file.
  if (!mundo.write(ui->treeWidget))
    return;

  // Check if gazebo is running before opening the simulation
  checkGazeboIsRunning();

  QString gazeboProcessCmd = "roslaunch";
  QStringList gazeboProcessArgs;
  gazeboProcessArgs << "Database" << launchFile;
  QString worldFilename = mundo.getWorld().filename();
  gazeboProcessArgs << QString("world:=%1").arg(worldFilename);

  gazeboProcessArgs << QString("control_strategy:=%1").arg(modelConfigFilePath);

  // Creates the process to execute the simulation
  QProcess* gazeboSimulationProcess = new QProcess(this);
  gazeboSimulationProcess->setWorkingDirectory(
      settings.getCatkinWorkspacePath());
  gazeboSimulationProcess->setProgram(gazeboProcessCmd);
  gazeboSimulationProcess->setArguments(gazeboProcessArgs);
  gazeboSimulationProcess->setProcessEnvironment(gazeboProcessEnv);
  ProcessOutputWindow* gazeboProcessWindow =
      new ProcessOutputWindow(gazeboSimulationProcess, this);
  gazeboProcessWindow->setProcessName(tr("Running Simulation"));
  gazeboProcessWindow->show();
  gazeboProcessWindow->start(gazeboProcessEnv);
  qDebug() << "Started process id: " << gazeboSimulationProcess->processId();

  ui->jointValuesPushButton->setDisabled(false);
}

void MainWindow::on_actionNew_triggered()
{
  // Get template directory path
  AppSettings settings;
  QString tiltProjectPath = settings.getTiltProjectPath();
  QString templatesFolderPath =
      QDir::cleanPath(tiltProjectPath + QDir::separator() + "worlds" +
                      QDir::separator() + "templates");

  // Opens a dialog box to allow the user to select the template file
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Select New Simulation World Template"), templatesFolderPath,
      tr("World Template File (*.tpl)"));
  // Checks if the user cancelled the opertaion (ie. the filename is empty)
  if (filename.isEmpty())
    return;

  // Get the directory of the selected file
  QFileInfo templateFileInfo(filename);
  QString dir = templateFileInfo.absoluteDir().absolutePath();

  // colocando imagem do mundo fornecida pelo usuário  na interface gráfica
  QString imagefile(QDir::cleanPath(dir + QDir::separator() + "imagem.gif"));
  QFileInfo fileInfo(imagefile);
  if (fileInfo.exists())
  {
    setWorldPreviewImage(imagefile);
  }

  // Populates the settings in the tree view
  ui->treeWidget->clear();
  mundo.loadWorld(filename, ui->treeWidget);

  // Enables menu functions
  ui->actionSave->setEnabled(true);
  ui->menuEdit->setEnabled(true);
  ui->actionSave->setEnabled(true);
  ui->menuEdit->setEnabled(true);
  ui->startGazeboPushButton->setEnabled(true);

  // Setup internal class state to indicate that a template was opened.
  istemplate = true;
  hilAsync = false;
}

void MainWindow::on_treeWidget_itemDoubleClicked(QTreeWidgetItem* item,
                                                 int column)
{
  // Allow edition of itens in the second column
  if (column == 1)
  {
    if (item->flags() & Qt::ItemIsEditable)
      ui->treeWidget->editItem(item, column);
  }

  // If the text in the first column is not URI, there is no allowed action.
  if (item->text(0).toLower() != "uri")
  {
    return;
  }

  // Shows the Model Setup Dialog if the user clicked in a URI item in the
  // first column.
  if (column == 0)
  {
    AppSettings settings;
    QString gazeboModelPath = settings.getGazeboModelPath();

    // Model description file
    QStringList modelURI = item->text(1).split("//");
    if (modelURI.length() < 2)
    {
      showInvalidURIMessage();
      return;
    }
    QString modelName = modelURI.at(1);

    QString localModelPath = QDir::cleanPath(
        gazeboModelPath + QDir::separator() + modelName + QDir::separator() +
        "robot" + QDir::separator() + "model.sdf");
    QFileInfo modelFileInfo(localModelPath);
    QDir modelDir = modelFileInfo.absoluteDir();
    // Access model parent dir
    modelDir.cdUp();

    // Controller configuration file path
    QString localConfig =
        QDir::cleanPath(modelDir.absolutePath() + QDir::separator() + "config" +
                        QDir::separator() + "config.xml");

    // Check if the SDF and config.xml files exists
    QFileInfo localConfigInfo(localConfig);

    if (!localConfigInfo.exists())
    {
      QString modelName = getModelName(item);
      QMessageBox::critical(this, tr("Error"),
                            tr("Error while trying to open the model "
                               "config.xml file.\nPlease make sure that "
                               "the model has a directory named config "
                               "under its root directory and a config.xml "
                               "file."));
      qCritical("%s%s. %s",
                qUtf8Printable(tr("Error while trying to find the "
                                  "config.xml for the ")),
                qUtf8Printable(modelName),
                qUtf8Printable(tr("Please make sure this model exists "
                                  "and has a config.xml file.")));
      return;
    }
    if (!modelFileInfo.exists())
    {
      QMessageBox::critical(this, tr("Error"),
                            tr("Error while trying to open the SDF file "
                               "for the selected model. Please make sure "
                               "the file exists, is under the correct"
                               " path and is named robot.sdf."));
      qCritical("Error while trying to open the SDF file for the "
                "selected model. Please make sure this model exists.");
      return;
    }

    // Opens the window
    ModelSetupDialog modelsetup;
    modelsetup.setStepTime(getStepTimeUs());
    modelsetup.setModel(localModelPath, localConfig);

    modelsetup.setModal(true);
    modelsetup.exec();

    // Checks if hill is enabled
    hilAsync = modelsetup.hilAsync();
    if (hilAsync)
    {
      /* Set adequate values to allow correct working of the HIL
       * simulation.
       */
      for (int i = 0; i < ui->treeWidget->topLevelItemCount(); i++)
      {
        QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
        if (item->text(0) == "Physics")
        {
          QTreeWidgetItemIterator it(item);
          while (*it)
          {
            // Set step time
            if ((*it)->text(0) == "Step time")
            {
              (*it)->setText(1, "0.004");
            }
            // Set the real time factor
            else if ((*it)->text(0) == "Real time factor")
            {
              (*it)->setText(1, "1");
            }
            // Set the real time update rate
            else if ((*it)->text(0) == "Real time update rate")
            {
              (*it)->setText(1, "250");
            }
            ++it;
          }
        }
        else if (item->text(0) == "Plugin")
        {
          //! @todo Check the behavior of this part with Arthur
          //! This action was modified because it caused the
          //! application to crash after closing the modelsetupdialog.
          // item->child(2)->setText(1, QString("hilAsync"));

          QTreeWidgetItemIterator it(item);
          while (*it)
          {
            if ((*it)->text(0) == "ok")
            {
              (*it)->setText(1, "hilAsync");
            }
            ++it;
          }
        }
      }
    }
    else
    {
      // Disables the hil
      for (int i = 0; i < ui->treeWidget->topLevelItemCount(); i++)
      {
        QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
        if (item->text(0) == "Plugin")
        {
          QTreeWidgetItemIterator it(item);

          while (*it)
          {
            if ((*it)->text(0) == "ok")
            {
              (*it)->setText(1, "nothil");
            }
            ++it;
          }
        }
      }
    }
  }
}

void MainWindow::on_treeWidget_itemClicked(QTreeWidgetItem* item, int column)
{
  Q_UNUSED(column)

  if (item->text(0).toLower() == "uri")
  {
    AppSettings settings;
    QString gazeboModelPath = settings.getGazeboModelPath();
    // Split the selected model URI
    QStringList list = item->text(1).split("//");
    if (list.length() < 2)
    {
      showInvalidURIMessage();
      return;
    }
    QString currentModelname = list.at(1);

    QDir modelDir(QDir::cleanPath(gazeboModelPath + QDir::separator() +
                                  currentModelname));
    QFileInfo imageFileInfo(modelDir.absoluteFilePath("imagem.gif"));

    // Checks if there exists an image for the current model
    if (imageFileInfo.exists())
    {
      QGraphicsScene* scene = new QGraphicsScene(ui->modelGraphicsView);
      scene->addPixmap(QPixmap(imageFileInfo.absoluteFilePath()));
      ui->modelGraphicsView->setScene(scene);
      ui->modelGraphicsView->show();
    }
    else
    {
      QGraphicsScene* scene = new QGraphicsScene(ui->modelGraphicsView);
      ui->modelGraphicsView->setScene(scene);
      ui->modelGraphicsView->show();
    }
  }

  // Enable or disable the remove pushbutton
  const bool isInclude = item->text(0).toLower() == "include";
  ui->removePushButton->setEnabled(isInclude);
}

void MainWindow::on_actionOpen_triggered()
{
  AppSettings settings;
  QString worldsPackagePath = settings.getWorldsPackagePath();
  // Open dialog to select the world file
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Open World"), worldsPackagePath, tr("World Files (*.world)"));
  // Check if the user cancelled the operation (i.e. the filename is empty)
  if (filename.isEmpty())
    return;

  // Get the directory of the file
  QFileInfo worldFileInfo(filename);
  QString dir = worldFileInfo.absoluteDir().absolutePath();

  // Get the file path for the scenary image
  QString imagefile(QDir::cleanPath(dir + QDir::separator() + "imagem.gif"));
  QFileInfo imageFileInfo(imagefile);
  if (imageFileInfo.exists())
  {
    QGraphicsScene* scene = new QGraphicsScene(ui->worldGraphicsView);
    scene->addPixmap(QPixmap(imagefile));
    ui->worldGraphicsView->setScene(scene);
    ui->worldGraphicsView->show();
  }
  // Clean the display widget tree
  ui->treeWidget->clear();
  // Adds new world information
  mundo.loadWorld(filename, ui->treeWidget);

  // Enables the menu itens
  hilAsync = false;
  ui->actionSave->setEnabled(true);
  ui->menuEdit->setEnabled(true);
  ui->actionSave->setEnabled(true);
  ui->menuEdit->setEnabled(true);
  ui->startGazeboPushButton->setEnabled(true);
  istemplate = false;
}

bool MainWindow::saveAs()
{
  AppSettings settings;
  QString worldsFolderPath = settings.getWorldsPackagePath();

  // Open a dialog to let the user select the save file name
  QString filename = QFileDialog::getSaveFileName(
      this, tr("Save World"), worldsFolderPath, tr("World Files (*.world)"));

  // Check if the user canceled the operation (i.e. the filename is empty)
  if (filename.isEmpty())
    return false;

  // Ensure that the file is terminated with a .world suffix
  if (!filename.endsWith(".world"))
  {
    filename += ".world";
  }

  auto world = mundo.getWorld();
  world.setFilename(filename);
  mundo.setWorld(world);

  // Update file
  mundo.write(ui->treeWidget);
  // Update template state
  istemplate = false;
  return true;
}

void MainWindow::on_actionExit_triggered()
{
  this->close();
}

void MainWindow::on_actionNewModel_triggered()
{
  // Get the list of models
  const ModelList models{};

  // Check if a robot model already exists in the GUI
  QTreeWidgetItemIterator it(ui->treeWidget);
  while (*it)
  {
    if ((*it)->text(0).toLower() == "include")
    {
      // Find uri
      QTreeWidgetItemIterator uriIt((*it));
      while (*uriIt)
      {
        if ((*uriIt)->text(0).toLower() == "uri")
        {
          const auto uriSplit = (*uriIt)->text(1).split("://");
          if (uriSplit.length() < 2)
            continue;

          if (models.isAModel(uriSplit.at(1)))
          {
            QMessageBox::critical(this, tr("Error"),
                                  tr("A robot model is already included in "
                                     "this simulation, and only one robot "
                                     "model can be included in a "
                                     "simulation.\n"
                                     "If you want to change the simulation "
                                     "model, please remove the %1 model before "
                                     "adding a new model.")
                                      .arg(uriSplit.at(1)));
            return;
          }
        }

        ++uriIt;
      }
    }

    ++it;
  }

  DialogNewModel newmodel(ui, this);
  newmodel.setModal(true);
  newmodel.exec();
}

void MainWindow::on_actionSave_triggered()
{
  saveAs();
}

void MainWindow::on_actionAbout_triggered()
{
  AboutDialog newform(this);
  newform.setModal(true);
  newform.exec();
}

void MainWindow::on_jointValuesPushButton_clicked()
{
  QMessageBox::information(this, tr("Functionality not implemented"),
                           tr("Dear user, the starting values for the model "
                              "joints cannot be selected in the GUI at this "
                              "point, check again in future versions to "
                              "check that this function has been "
                              "implemented!"));
  /*
   * Note: This functionallity was never correctly implemented.
   * As a limitation of time, I included a message informing this and
   * commented out the original code that is shown below this message.
   */
  //    JointsDialog newform(this);
  //    newform.setModal(true);
  //    newform.exec();
  //    ui->jointValuesPushButton->setDisabled(true);
}

void MainWindow::on_actionOptions_triggered()
{
  ApplicationSettingsDialog dialog(this);
  dialog.exec();
}

QString MainWindow::getModelName(QTreeWidgetItem* item)
{
  QTreeWidgetItem* parent = item->parent();
  if (parent != nullptr)
  {
    for (int i = 0; i < parent->childCount(); i++)
    {
      QTreeWidgetItem* child = parent->child(i);
      if (child->text(0).toLower() == "name")
      {
        return child->text(1);
      }
    }
  }
  return QString();
}

void MainWindow::checkROSEnvironment()
{
  const auto val = qgetenv("ROS_PACKAGE_PATH");
  const auto paths = QString(val).split(":");

  // Count valid paths
  int validPaths = 0;
  foreach (const QString& path, paths)
  {
    if (path.isEmpty())
      continue;

    QDir currentDir(path);
    if (currentDir.exists())
      validPaths++;
  }

  if (validPaths < 1)
  {
    QMessageBox::critical(this, tr("ROS is not configured correctly"),
                          tr("The path to the ROS applications was not found "
                             "in the current path. Please verify your .bashrc "
                             "file and ensure that both ROS and your catkin "
                             "workspace are sourced within the file. After "
                             "that, restart your terminal and open the ProVANT "
                             "Simulator GUI again."));

    ui->actionNew->setEnabled(false);
    ui->actionOpen->setEnabled(false);
  }
}

void MainWindow::setWorldPreviewImage(const QString& path)
{
  QGraphicsScene* scene = new QGraphicsScene(ui->worldGraphicsView);
  scene->addPixmap(QPixmap(path));
  ui->worldGraphicsView->setScene(scene);
  ui->worldGraphicsView->show();
}

void MainWindow::showInvalidURIMessage()
{
  QMessageBox::critical(this, tr("Error"),
                        tr("Invalid model URI selected. Please check "
                           "its value and try again."));
}

void MainWindow::checkGazeboIsRunning()
{
  CheckAndCloseProcess gz;
  gz.addProcess("gzclient");
  gz.addProcess("gzserver");

  if (gz.isProcessRunning())
  {
    QString msg = tr("Gazebo is running processes with the following "
                     "pids: "
                     "\n");

    QSet<int> pids = gz.getPids();
    for (auto it = pids.cbegin(); it != pids.cend(); ++it)
    {
      msg += QString::number(*it) + "\n";
    }

    msg += tr("\nStarting a new simulation with theses process open may "
              "result in an error.\n");
    msg += tr("Do you wish to close these processes before starting a "
              "new simulation?\n");

    QMessageBox::StandardButton res;
    res = QMessageBox::question(this, tr("Gazebo is Running"), msg);
    if (res == QMessageBox::Yes)
    {
      gz.closeProcesses();
    }
  }
}

uint64_t MainWindow::getStepTimeUs(bool* found) const
{
  // Find the physics item
  QTreeWidgetItem* physicsItem = nullptr;
  for (int i = 0; i < ui->treeWidget->topLevelItemCount(); i++)
  {
    if (ui->treeWidget->topLevelItem(i)->text(0).toLower() == "physics")
    {
      physicsItem = ui->treeWidget->topLevelItem(i);
      break;
    }
  }

  if (physicsItem == nullptr)
  {
    if (found != nullptr)
      *found = false;
    return 0;
  }

  // Find the step time item
  QTreeWidgetItem* stepTimeItem = nullptr;
  for (int i = 0; i < physicsItem->childCount(); i++)
  {
    if (physicsItem->child(i)->text(0).toLower() == "step time")
    {
      stepTimeItem = physicsItem->child(i);
      break;
    }
  }

  if (stepTimeItem == nullptr)
  {
    if (found != nullptr)
      *found = false;
    return 0;
  }

  bool ok = false;
  double stepTimeDouble = stepTimeItem->text(1).toDouble(&ok);
  if (!ok)
  {
    if (found != nullptr)
      *found = false;
    return 0;
  }

  if (found != nullptr)
    *found = true;
  return static_cast<uint64_t>(round(stepTimeDouble * 1e6));
}

void MainWindow::on_removePushButton_clicked()
{
  const auto selectedItems = ui->treeWidget->selectedItems();
  foreach (QTreeWidgetItem* item, selectedItems)
  {
    if (item->text(0).toLower() == "include")
    {
      delete item;
    }
  }
}
