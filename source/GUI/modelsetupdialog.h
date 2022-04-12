#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QTime>
#include <QtSerialPort/QtSerialPort>

#include "Business/model.h"
#include "Business/controller.h"

namespace Ui
{
class ModelSetupDialog;
}

/*!
 * \brief The ModelSetupDialog class is a window that allows the user to
 * configure a model used in the simulation.
 *
 * Among the options provided to the user, the following itens can be changed:
 *  - Control strategy;
 *  - Use of hardware in the loop simulation;
 *  - Sensors and actuators provided by the model;
 *  - Sampling time;
 *  - Initial pose;
 *  - Log files for the simulation;
 */
class ModelSetupDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ModelSetupDialog(QWidget* parent = 0);
  ~ModelSetupDialog();

  //! Sets the controller and model displayed on the window.
  void setModel(QString modelFile, QString controllerFile);
  void setStepTime(uint64_t us);

  //! Returns the state of the hardware on the loop flag.
  bool hilSync() const;
  bool hilAsync() const;

signals:
  /**
   * @brief hilFlagChanged Singla emited when the hardware on the loop status
   * flag changes.
   * @param hil The status of the hardware on the loop flag.
   */
  void hilFlagChanged(bool hil);

public slots:
  //! Save the controller data in the respective file.
  void saveConfig();

  //! Saves the modifications made to the model and closes the window.
  void accept();

  //! Defines the state of the hardware on the loop flag.
  void setHilSync(bool hilSync);
  void setHilAsync(bool hilAsync);

  //! Lists the available control strategies in the source directory and
  //! shows then in the control selection combobox.
  void updateAvailableControllers();

private slots:
  //! Creates a new controll strategy.
  void on_newControllerButton_clicked();

  //! Compiles the selected controll strategy.
  void on_compileControllerButton_clicked();

  //! Opens an existing controll strategy project.
  void on_openControllerButton_clicked();

  //! Add a new sensor to the model.
  void on_addSensorButton_clicked();

  //! Removes the selected sensor from the model.
  void on_removeSensorButton_clicked();

  //! Add an actuator to the model.
  void on_addActuatorButton_clicked();

  //! Removes the selected actuator from the model.
  void on_removeActuatorButton_clicked();

  //! Enables the Hardware on  the Loop mode.
  void on_hilCheckBoxSync_clicked(bool checked);
  void on_hilCheckBoxAsync_clicked(bool checked);
  void on_compileHilButton_clicked();

  /**
   * @brief on_refreshUSB_clicked Method called when the user clicks on the
   * Refresh button under the HIL tab. This method clears the current list of
   * USB ports, and queries the system for the list of avialble serial devices
   * connected to it, then formats into a suitable text for UI presentation.
   *
   * If no devices are found connected to the system, the port selectors are
   * disabled.
   */
  void on_refreshUSB_clicked();

  /**
   * @brief on_selectUSB1_currentIndexChanged Method called when the user change
   * the USB port selected in the USART1 combo box.
   *
   * @param index Index of the selected item in the USART1 combo box.
   */
  void on_selectUSB1_currentIndexChanged(int index);
  /**
   * @brief on_selectUSB2_currentIndexChanged Method called when the user change
   * the USB port selected in the USART2 combo box.
   *
   * @param index Index of the selected item.
   */
  void on_selectUSB2_currentIndexChanged(int index);

  /**
   * @brief on_turbulanceCheckBox_clicked Method called when the user changes
   * the selected turbulence model.
   */
  void on_turbulanceCheckBox_clicked();

  /**
   * @brief on_treeWidget_itemDoubleClicked Method called when the user double
   * clicks on an item under the model parameters tab.
   *
   * This method is used to set the correct item flags in order to ensure the
   * user can edit the text of the selected item.
   *
   * @param item Pointer to the item which the user double taped.
   * @param column Index of the column where the user cliked.
   */
  void on_treeWidget_itemDoubleClicked(QTreeWidgetItem* item, int column);

protected:
  //! Pointer the elements of the user interface
  Ui::ModelSetupDialog* ui;

  /**
   * @brief calcSimulationSteps
   * Calculate how many simulation steps the simulation must last based on
   * the duration specified in the GUI and the specified step time.
   * @return Number of steps the simulation must be executed for.
   */
  uint64_t calcSimulationSteps() const;

  void fillBaudrateSelector();

private:
  //! Used to manipulate the model config.xml file
  Model _model;
  //! Used to manipulate the controller options
  Controller _controller;

  //! Indiciates if the model should use the hardware on the loop mode.
  bool _hilAsync = false;
  bool _hilSync = false;

  //! Step time of the simulation. Used in the calculation of the simulation
  //! duration.
  uint64_t _stepTimeUs;
};

#endif  // DIALOG_H
