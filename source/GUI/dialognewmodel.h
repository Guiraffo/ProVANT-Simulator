#ifndef DIALOGNEWMODEL_H
#define DIALOGNEWMODEL_H

#include <QDialog>

#include "mainwindow.h"

namespace Ui
{
class Dialognewmodel;
}

/**
 * @brief The DialogNewModel class Shows a dialog box that allows the user to
 * select a model and isert it in the world configuration.
 *
 * @todo Refactor this class to use a signal in the main window to add the new
 * model and thus remove depency on the main window pointer.
 * @todo Unify the method parsePoseVector with the method splitVector in the
 * ModelConfigFile class as they perform the exact same function.
 */
class DialogNewModel : public QDialog
{
  Q_OBJECT
public:
  /**
   * @brief DialogNewModel Creates and configures a new DialogNewModel
   * object.
   * @param mainWindow Pointer to the MainWindow user interface elements.
   * @param parent Pointer to the paret widget.
   */
  explicit DialogNewModel(Ui::MainWindow* mainWindow, QWidget* parent = 0);
  /**
   * @brief ~Dialognewmodel Destroys the user interface.
   */
  virtual ~DialogNewModel();

private slots:
  /**
   * @brief on_buttonBox_accepted Called when the dialog box is accepted.
   * Add the selected model to the world configuration.
   */
  void on_buttonBox_accepted();

protected:
  /**
   * @brief findModels Search trough the directories under the gazebo model
   * path for folder of models.
   *
   * Every model that has a config.xml file and a SDF file is added to the
   * selection combobox.
   */
  void populateModelList();

private:
  /**
   * @brief parsePoseVector Parse the pose elements from a SDF string from a
   * pose element and creates appropriate elements in the tree widget to allow
   * configuration of those values.
   *
   * @param data Pose string.
   * @param element Tree widget item to add the child itens.
   */
  void parsePoseVector(const QString& data, QTreeWidgetItem* element);

  //! Pointer to the user interface elemetns.
  Ui::Dialognewmodel* ui;

  //! Pointer to the mainwindow UI elements.
  Ui::MainWindow* parentUi;
};

#endif  // DIALOGNEWMODEL_H
