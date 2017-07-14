#ifndef DIALOGSENSORS_H
#define DIALOGSENSORS_H

#include <QDialog>
#include"dialog.h"
#include"QString"

namespace Ui {
class DialogSensors;
}

class DialogSensors : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSensors(Model,QWidget *parent = 0);
    ~DialogSensors();

    int n;
    std::vector<QString> values;
    QString type;
private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogSensors *ui;

    Model registry;
};

#endif // DIALOGSENSORS_H
