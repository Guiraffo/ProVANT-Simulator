#ifndef DIALOGIMU_H
#define DIALOGIMU_H

#include <QDialog>
#include"Business/model.h"

namespace Ui {
class DialogIMU;
}

class DialogIMU : public QDialog
{
    Q_OBJECT

public:
    explicit DialogIMU(Model model, QWidget *parent = 0);
    ~DialogIMU();
    QString link;
    QString topic;
    QString noise;
private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogIMU *ui;
    Model registry;

};

#endif // DIALOGIMU_H
