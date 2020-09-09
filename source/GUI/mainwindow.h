/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the MainWindow GUI class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"

#include "Business/world.h"

namespace Ui {
class MainWindow;
}

/**
 * @brief The MainWindow class is the starting point for the ProVANT simulator
 * GUI.
 *
 * This class shows a tree widget item that will display every element
 * contained in the description of the Gazebo simulation world, and allow the
 * user to insert new models, edit the options for the existing ones,
 * visualize a representation of the scenario and the selected models and
 * finally, to start the Gazebo simulation.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief MainWindow Creates a new maindow object.
     * @param parent Parent widget.
     */
    explicit MainWindow(QWidget *parent = nullptr);

    /**
      * @brief ~MainWindow Destroys the current object.
      *
      * Free the pointer to the elements in the graphical user interface.
      */
    ~MainWindow();

private slots:
    /**
     * @brief on_startGazeboPushButton_clicked Method called when the start
     * gazebo pushbutton is clicked.
     *
     * Starts the simulation using roslaunch and the paramters configured in the
     * GUI.
     */
    void on_startGazeboPushButton_clicked();

    /**
     * @brief on_actionNew_triggered Method called when the New action on the
     * File menu is triggered.
     *
     * Allows the creation of a new world based on a provided template.
     */
    void on_actionNew_triggered();

    /**
     * @brief on_treeWidget_itemDoubleClicked Method called when the user double
     * clicks on top of a tree widget item in the simulation paramters display.
     *
     * Checks if the double clicked item is of the type URI, and in that casem
     * opens the model setup dialog that allows the user to configure the
     * options for the selected model.
     *
     * @param item Item which the user double clicked.
     * @param column Colum where the user double clicked.
     */
    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);

    /**
     * @brief on_treeWidget_itemClicked Method called when the user clicks on a
     * tree item in the simulation display widget.
     *
     * Check if the selected item is a model, and in that case shows an image of
     * the model in the GUI.
     *
     * @param item Selected tree item
     * @param column Column which the user clicked
     */
    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

    /**
     * @brief on_actionOpen_triggered Method called when the Open action in the
     * file menu is triggered.
     *
     * Shows a file browser dialog and allows the user to select and open a
     * simulation world.
     */
    void on_actionOpen_triggered();

    /**
     * @brief saveAs Method called to save the current simulation parameters in
     * a new world file.
     *
     * @return True if the file was saved and false otherwise.
     */
    bool saveAs();

    /**
     * @brief on_actionSave_triggered Method called when the Save action on the
     * file menu is triggered.
     *
     * Shows a file browser and allows the user to save the current simulation
     * parameters in a new world file.
     *
     * @see saveAs().
     */
    void on_actionSave_triggered();

    /**
     * @brief on_actionExit_triggered Method called when the Exit action on the
     * file menu is triggered.
     *
     * Closes the application.
     */
    void on_actionExit_triggered();

    /**
     * @brief on_actionNewModel_triggered Method called when the New Model
     * action under Edit -> Model is triggered.
     *
     * Shows a dialog box that allows the user to select a new model to include
     * in the simulation.
     */
    void on_actionNewModel_triggered();

    /**
     * @brief on_actionAbout_triggered Method called when the About ProVANT
     * simulator action under the About menu is triggered.
     *
     * Shows a dialog box with additional information about the ProVANT
     * simulator.
     */
    void on_actionAbout_triggered();

    /**
     * @brief on_jointValuesPushButton_clicked Method called when the Initial
     * values of joints pushbutton is clicked.
     *
     * Shows a dialog box that allows the user to select the initial values
     * of the model joints.
     *
     * @todo Refactor this. This cannot easily be implemented as a global
     * method, the easier option to implement this is as a paramters in the
     * model setup dialog.
     */
    void on_jointValuesPushButton_clicked();

    /**
     * @brief on_actionOptions_triggered Method called when the action Options
     * on the Tools menu is clicked.
     *
     * Shows a dialog box that allows the user to set the values for several
     * configuration options used troughout the ProVANT simulator.
     */
    void on_actionOptions_triggered();

protected:
    //! Pointer to the elements in the graphical user interface form
    Ui::MainWindow *ui;

    /**
     * @brief getModelName Returns the model name that is closest to an item
     * in the simulation parameters tree.
     *
     * @param item Item to begin the search for the model name.
     * @return Model name, or an empty string if a parent model name is not
     * found.
     */
    QString getModelName(QTreeWidgetItem *item);

    /**
     * @brief setWorldPreviewImage Sets the world preview image in the GUI.
     * @param path Path to the image file.
     */
    void setWorldPreviewImage(const QString &path);

    /**
     * @brief showInvalidURIMessage Shows a QMessageBox indicating that the
     * currently selected URI item is invalid.
     */
    void showInvalidURIMessage();

private:
    //! Class for reading and writing data relative to the simulation world.
    world mundo;
    //! Indicates if the currently selected simulation world is a template.
    bool istemplate;
    //! Indicates if the current simulation is using hardware in the loop
    bool hil = false;
};

#endif // MAINWINDOW_H
