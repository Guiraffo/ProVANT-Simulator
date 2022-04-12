#ifndef APPLICATIONSETTINGSDIALOG_H
#define APPLICATIONSETTINGSDIALOG_H

#include <QDialog>

namespace Ui
{
class ApplicationSettingsDialog;
}

/*!
 * \brief The ApplicationSettingsDialog class implements a dialog window which
 * allows the user to set the system paths in a graphical interface.
 *
 * Thw window is constitued of a label showing the parameter being configured
 * followed by a FileBrowserWidget which allows the user to browse to its file
 * system and find the appropriate path.
 *
 * The design of the class reads and sets the values of the configuration
 * parameters utilizing the methods of the AppSettings class.
 *
 * To avoid unecessary error messages when opening this window, the unchecked
 * methods for retrieving the values are used.
 *
 * The following itens should be taken in consideration when adding new
 * configurations to this class:
 *
 * * Each parameter should be composed by a label informing the user of the
 * parameter bening modified and a FileBrowserWidget configurated to locate
 * the apropriate file type.
 * * Each parameter should have a tooltip informing what the parameter is, and
 * providing a good sugestion for the location of the file.
 */
class ApplicationSettingsDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ApplicationSettingsDialog(QWidget* parent = nullptr);
  ~ApplicationSettingsDialog();

private:
  //! The pointer to objects contained in the user interface of this class.
  Ui::ApplicationSettingsDialog* ui;

  void showErrorOnSettingMessage(const QString& key) const;
public slots:
  void accept();
  void reject();
};

#endif  // APPLICATIONSETTINGSDIALOG_H
