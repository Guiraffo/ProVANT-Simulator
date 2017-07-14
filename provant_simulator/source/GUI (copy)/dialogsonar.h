#ifndef DIALOGSONAR_H
#define DIALOGSONAR_H

#include <QDialog>
#include "Business/model.h"

namespace Ui {
class DialogSonar;
}

class DialogSonar : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSonar(Model,QWidget *parent = 0);
    ~DialogSonar();
    QString max;
    QString min;
    QString radius;
    QString link;
    QString topic;
    Model registry;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogSonar *ui;
};

#endif // DIALOGSONAR_H
