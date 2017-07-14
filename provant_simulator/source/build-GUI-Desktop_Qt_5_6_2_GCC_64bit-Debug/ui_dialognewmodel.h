/********************************************************************************
** Form generated from reading UI file 'dialognewmodel.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOGNEWMODEL_H
#define UI_DIALOGNEWMODEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>

QT_BEGIN_NAMESPACE

class Ui_Dialognewmodel
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *label;
    QComboBox *comboBox;

    void setupUi(QDialog *Dialognewmodel)
    {
        if (Dialognewmodel->objectName().isEmpty())
            Dialognewmodel->setObjectName(QStringLiteral("Dialognewmodel"));
        Dialognewmodel->resize(277, 149);
        buttonBox = new QDialogButtonBox(Dialognewmodel);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(50, 90, 181, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        label = new QLabel(Dialognewmodel);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(30, 20, 131, 17));
        comboBox = new QComboBox(Dialognewmodel);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(30, 50, 231, 27));

        retranslateUi(Dialognewmodel);
        QObject::connect(buttonBox, SIGNAL(accepted()), Dialognewmodel, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), Dialognewmodel, SLOT(reject()));

        QMetaObject::connectSlotsByName(Dialognewmodel);
    } // setupUi

    void retranslateUi(QDialog *Dialognewmodel)
    {
        Dialognewmodel->setWindowTitle(QApplication::translate("Dialognewmodel", "Dialog", 0));
        label->setText(QApplication::translate("Dialognewmodel", "Choose the model", 0));
    } // retranslateUi

};

namespace Ui {
    class Dialognewmodel: public Ui_Dialognewmodel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGNEWMODEL_H
