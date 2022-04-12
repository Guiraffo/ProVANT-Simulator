#include "dialognewcontroller.h"
#include "ui_dialognewcontroller.h"

#include "DataAccess/ControllerElements/newstrategy.h"

/*!
 * \brief DialogNewController::DialogNewController
 * \param parent Window wich contains this dialog.
 *
 * Initialize a new DialogNewController object and setup the user interface.
 */
DialogNewController::DialogNewController(QWidget* parent)
  : QDialog(parent), ui(new Ui::DialogNewController)
{
  ui->setupUi(this);
}

/*!
 * \brief DialogNewController::~DialogNewController
 * Deletes the objects contained in the user interface.
 */
DialogNewController::~DialogNewController()
{
  delete ui;
}

/*!
 * \brief DialogNewController::accept
 *
 * Create a new controller strategy and calls the base class accept method
 * to allow for correct propagation of signals and other behaviors.
 *
 * @todo Refactor this method to force verification of the ROS naming
 * conventions for a new package. For now, this method only foces that the
 * package name be lowercase.
 */
void DialogNewController::accept()
{
  NewStrategy::createProject(ui->lineEdit->text().toLower());
  QDialog::accept();
}
