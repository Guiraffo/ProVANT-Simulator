/********************************************************************************
** Form generated from reading UI file 'dialognewcontroller.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOGNEWCONTROLLER_H
#define UI_DIALOGNEWCONTROLLER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>

QT_BEGIN_NAMESPACE

class Ui_DialogNewController
{
public:
    QDialogButtonBox *buttonBox;
    QLineEdit *lineEdit;
    QLabel *label;

    void setupUi(QDialog *DialogNewController)
    {
        if (DialogNewController->objectName().isEmpty())
            DialogNewController->setObjectName(QStringLiteral("DialogNewController"));
        DialogNewController->resize(300, 139);
        buttonBox = new QDialogButtonBox(DialogNewController);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(100, 90, 181, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        lineEdit = new QLineEdit(DialogNewController);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setGeometry(QRect(20, 50, 261, 27));
        label = new QLabel(DialogNewController);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 20, 241, 17));

        retranslateUi(DialogNewController);
        QObject::connect(buttonBox, SIGNAL(accepted()), DialogNewController, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), DialogNewController, SLOT(reject()));

        QMetaObject::connectSlotsByName(DialogNewController);
    } // setupUi

    void retranslateUi(QDialog *DialogNewController)
    {
        DialogNewController->setWindowTitle(QApplication::translate("DialogNewController", "Dialog", 0));
        label->setText(QApplication::translate("DialogNewController", "Choose the name of new Controller: ", 0));
    } // retranslateUi

};

namespace Ui {
    class DialogNewController: public Ui_DialogNewController {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGNEWCONTROLLER_H
