#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "Business/model.h"
#include "Business/controller.h"

namespace Ui {
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
    explicit ModelSetupDialog(QWidget *parent = 0);
    ~ModelSetupDialog();

    //! Sets the controller and model displayed on the window.
    void setModel(QString modelFile, QString controllerFile);

    //! Returns the state of the hardware on the loop flag.
    bool hil() const;

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
    void setHil(bool hil);

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
    void on_hilCheckBox_clicked(bool checked);

    void on_turbulanceCheckBox_clicked();

protected:
    //! Pointer the elements of the user interface
    Ui::ModelSetupDialog *ui;

private:
    //! Used to manipulate the model config.xml file
    Model _model;
    //! Used to manipulate the controller options
    Controller _controller;

    //! Indiciates if the model should use the hardware on the loop mode.
    bool _hil = false;
};

#endif // DIALOG_H
