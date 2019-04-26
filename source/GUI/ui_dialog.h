/********************************************************************************
** Form generated from reading UI file 'dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_H
#define UI_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QDialogButtonBox *buttonBox;
    QTabWidget *tabWidget;
    QWidget *tab;
    QTreeWidget *treeWidget;
    QWidget *tab_2;
    QLabel *label;
    QComboBox *comboBox;
    QPushButton *pushButton_5;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_11;
    QLineEdit *SampleEdit;
    QLineEdit *ErrorEdit;
    QLineEdit *ReferenceEdit;
    QLineEdit *SensorEdit;
    QLineEdit *ActuatorEdit;
    QPushButton *pushButton_6;
    QPushButton *pushButton_7;
    QCheckBox *checkBox;
    QWidget *tab_3;
    QListWidget *listWidget_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QWidget *tab_4;
    QListWidget *listWidget;
    QPushButton *pushButton;
    QPushButton *pushButton_2;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QStringLiteral("Dialog"));
        Dialog->resize(605, 487);
        buttonBox = new QDialogButtonBox(Dialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(480, 20, 81, 241));
        buttonBox->setOrientation(Qt::Vertical);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        tabWidget = new QTabWidget(Dialog);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(20, 10, 421, 451));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        treeWidget = new QTreeWidget(tab);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QStringLiteral("Item"));
        treeWidget->setHeaderItem(__qtreewidgetitem);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        treeWidget->setGeometry(QRect(10, 10, 391, 401));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        label = new QLabel(tab_2);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 40, 71, 31));
        comboBox = new QComboBox(tab_2);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(100, 40, 301, 27));
        pushButton_5 = new QPushButton(tab_2);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));
        pushButton_5->setGeometry(QRect(10, 80, 121, 27));
        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(20, 260, 91, 31));
        label_3 = new QLabel(tab_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(20, 290, 91, 31));
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(20, 320, 101, 31));
        label_5 = new QLabel(tab_2);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(20, 350, 111, 31));
        label_11 = new QLabel(tab_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(20, 380, 121, 31));
        SampleEdit = new QLineEdit(tab_2);
        SampleEdit->setObjectName(QStringLiteral("SampleEdit"));
        SampleEdit->setGeometry(QRect(150, 260, 251, 27));
        ErrorEdit = new QLineEdit(tab_2);
        ErrorEdit->setObjectName(QStringLiteral("ErrorEdit"));
        ErrorEdit->setGeometry(QRect(150, 290, 251, 27));
        ReferenceEdit = new QLineEdit(tab_2);
        ReferenceEdit->setObjectName(QStringLiteral("ReferenceEdit"));
        ReferenceEdit->setGeometry(QRect(150, 320, 251, 27));
        SensorEdit = new QLineEdit(tab_2);
        SensorEdit->setObjectName(QStringLiteral("SensorEdit"));
        SensorEdit->setGeometry(QRect(150, 350, 251, 27));
        ActuatorEdit = new QLineEdit(tab_2);
        ActuatorEdit->setObjectName(QStringLiteral("ActuatorEdit"));
        ActuatorEdit->setGeometry(QRect(150, 380, 251, 27));
        pushButton_6 = new QPushButton(tab_2);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));
        pushButton_6->setGeometry(QRect(150, 80, 121, 27));
        pushButton_7 = new QPushButton(tab_2);
        pushButton_7->setObjectName(QStringLiteral("pushButton_7"));
        pushButton_7->setGeometry(QRect(290, 80, 121, 27));
        checkBox = new QCheckBox(tab_2);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setGeometry(QRect(20, 10, 171, 22));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        listWidget_2 = new QListWidget(tab_3);
        listWidget_2->setObjectName(QStringLiteral("listWidget_2"));
        listWidget_2->setGeometry(QRect(10, 10, 391, 351));
        pushButton_3 = new QPushButton(tab_3);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(10, 380, 99, 27));
        pushButton_4 = new QPushButton(tab_3);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));
        pushButton_4->setGeometry(QRect(120, 380, 99, 27));
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        listWidget = new QListWidget(tab_4);
        listWidget->setObjectName(QStringLiteral("listWidget"));
        listWidget->setGeometry(QRect(10, 10, 391, 351));
        pushButton = new QPushButton(tab_4);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(10, 380, 99, 27));
        pushButton_2 = new QPushButton(tab_4);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(120, 380, 99, 27));
        tabWidget->addTab(tab_4, QString());

        retranslateUi(Dialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), Dialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), Dialog, SLOT(reject()));

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "Dialog", 0));
        QTreeWidgetItem *___qtreewidgetitem = treeWidget->headerItem();
        ___qtreewidgetitem->setText(1, QApplication::translate("Dialog", "Description", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("Dialog", "Parameters", 0));
        label->setText(QApplication::translate("Dialog", "<html><head/><body><p>Controller</p></body></html>", 0));
        pushButton_5->setText(QApplication::translate("Dialog", "New controller", 0));
        label_2->setText(QApplication::translate("Dialog", "<html><head/><body><p>Sample time</p></body></html>", 0));
        label_3->setText(QApplication::translate("Dialog", "<html><head/><body><p>Error file</p></body></html>", 0));
        label_4->setText(QApplication::translate("Dialog", "<html><head/><body><p>Reference file</p></body></html>", 0));
        label_5->setText(QApplication::translate("Dialog", "<html><head/><body><p>Sensor data file</p></body></html>", 0));
        label_11->setText(QApplication::translate("Dialog", "<html><head/><body><p>Actuator data file</p></body></html>", 0));
        pushButton_6->setText(QApplication::translate("Dialog", "Open Controller", 0));
        pushButton_7->setText(QApplication::translate("Dialog", "Compile", 0));
        checkBox->setText(QApplication::translate("Dialog", "Hardware-in-the-loop", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("Dialog", "Controller", 0));
        pushButton_3->setText(QApplication::translate("Dialog", "Add", 0));
        pushButton_4->setText(QApplication::translate("Dialog", "Remove", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("Dialog", "Sensors", 0));
        pushButton->setText(QApplication::translate("Dialog", "Add", 0));
        pushButton_2->setText(QApplication::translate("Dialog", "Remove", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("Dialog", "Actuators", 0));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_H
