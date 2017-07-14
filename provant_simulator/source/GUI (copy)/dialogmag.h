#ifndef DIALOGMAG_H
#define DIALOGMAG_H

#include <QDialog>
#include "Business/model.h"

namespace Ui {
class DialogMag;
}

class DialogMag : public QDialog
{
    Q_OBJECT

public:
    explicit DialogMag(Model model,QWidget *parent = 0);
    ~DialogMag();
    QString link;
    QString topic;
    QString noise;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::DialogMag *ui;
    Model registry;

};

#endif // DIALOGMAG_H
