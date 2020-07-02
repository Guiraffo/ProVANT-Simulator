#ifndef APPLICATIONSETTINGSDIALOG_H
#define APPLICATIONSETTINGSDIALOG_H

#include <QDialog>

namespace Ui {
class ApplicationSettingsDialog;
}

class ApplicationSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ApplicationSettingsDialog(QWidget *parent = nullptr);
    ~ApplicationSettingsDialog();

private:
    Ui::ApplicationSettingsDialog *ui;

    void showErrorOnSettingMessage(const QString &key) const;
public slots:
    void accept();
    void reject();
};

#endif // APPLICATIONSETTINGSDIALOG_H
