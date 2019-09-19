/********************************************************************************
** Form generated from reading UI file 'jointsdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOINTSDIALOG_H
#define UI_JOINTSDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTreeWidget>

QT_BEGIN_NAMESPACE

class Ui_JointsDialog
{
public:
    QDialogButtonBox *buttonBox;
    QTreeWidget *treeWidget;

    void setupUi(QDialog *JointsDialog)
    {
        if (JointsDialog->objectName().isEmpty())
            JointsDialog->setObjectName(QStringLiteral("JointsDialog"));
        JointsDialog->resize(400, 300);
        buttonBox = new QDialogButtonBox(JointsDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(290, 20, 81, 241));
        buttonBox->setOrientation(Qt::Vertical);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        treeWidget = new QTreeWidget(JointsDialog);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        treeWidget->setGeometry(QRect(20, 21, 211, 261));

        retranslateUi(JointsDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), JointsDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), JointsDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(JointsDialog);
    } // setupUi

    void retranslateUi(QDialog *JointsDialog)
    {
        JointsDialog->setWindowTitle(QApplication::translate("JointsDialog", "Dialog", 0));
        QTreeWidgetItem *___qtreewidgetitem = treeWidget->headerItem();
        ___qtreewidgetitem->setText(1, QApplication::translate("JointsDialog", "Value", 0));
        ___qtreewidgetitem->setText(0, QApplication::translate("JointsDialog", "Joint", 0));
    } // retranslateUi

};

namespace Ui {
    class JointsDialog: public Ui_JointsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOINTSDIALOG_H
