#ifndef DIALOGGPS_H
#define DIALOGGPS_H

#include <QDialog>
#include"Business/model.h"

namespace Ui {
class DialogGps;
}

class DialogGps : public QDialog
{
    Q_OBJECT

public:
    explicit DialogGps(Model,QWidget *parent = 0);
    ~DialogGps();
    QString link;
    QString topic;
    QString noise;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogGps *ui;
    Model registry;

};

#endif // DIALOGGPS_H
