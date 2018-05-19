/********************************************************************************
** Form generated from reading UI file 'dialogopencontroller.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOGOPENCONTROLLER_H
#define UI_DIALOGOPENCONTROLLER_H

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

class Ui_DialogOpenController
{
public:
    QDialogButtonBox *buttonBox;
    QComboBox *comboBox;
    QLabel *label;

    void setupUi(QDialog *DialogOpenController)
    {
        if (DialogOpenController->objectName().isEmpty())
            DialogOpenController->setObjectName(QStringLiteral("DialogOpenController"));
        DialogOpenController->resize(255, 163);
        buttonBox = new QDialogButtonBox(DialogOpenController);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(30, 100, 181, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        comboBox = new QComboBox(DialogOpenController);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(40, 60, 191, 27));
        label = new QLabel(DialogOpenController);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(40, 30, 131, 17));

        retranslateUi(DialogOpenController);
        QObject::connect(buttonBox, SIGNAL(accepted()), DialogOpenController, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), DialogOpenController, SLOT(reject()));

        QMetaObject::connectSlotsByName(DialogOpenController);
    } // setupUi

    void retranslateUi(QDialog *DialogOpenController)
    {
        DialogOpenController->setWindowTitle(QApplication::translate("DialogOpenController", "Dialog", 0));
        label->setText(QApplication::translate("DialogOpenController", "Select controller:", 0));
    } // retranslateUi

};

namespace Ui {
    class DialogOpenController: public Ui_DialogOpenController {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGOPENCONTROLLER_H
