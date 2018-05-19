/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionNew;
    QAction *actionSave;
    QAction *actionExit;
    QAction *actionAdd;
    QAction *actionDelete;
    QAction *actionChoose;
    QAction *actionParameters;
    QAction *actionStrategy_control;
    QAction *actionSensors;
    QAction *actionActuators;
    QAction *actionSave_2;
    QAction *actionSave_as;
    QAction *actionDelete_2;
    QAction *actionPhysics;
    QAction *actionGravity;
    QAction *actionParameters_2;
    QAction *actionSernsors;
    QAction *actionActuators_2;
    QAction *actionControl_Strategy;
    QAction *actionNew_2;
    QAction *actionRemove;
    QAction *actionSensors_2;
    QAction *actionActuators_3;
    QAction *actionControl_Strategy_2;
    QAction *actionEdit_link;
    QAction *actionOint;
    QAction *actionPlugin;
    QAction *actionEdit;
    QAction *actionOpen;
    QAction *actionAbout_ProVANT_Simulator;
    QWidget *centralWidget;
    QTreeWidget *treeWidget;
    QPushButton *pushButton;
    QLabel *label;
    QLabel *label_2;
    QGraphicsView *graphicsView;
    QGraphicsView *graphicsView_2;
    QLabel *label_3;
    QMenuBar *menuBar;
    QMenu *menuSimulation;
    QMenu *menuEdit;
    QMenu *menuModel;
    QMenu *menuAbout;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(939, 525);
        actionNew = new QAction(MainWindow);
        actionNew->setObjectName(QStringLiteral("actionNew"));
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionAdd = new QAction(MainWindow);
        actionAdd->setObjectName(QStringLiteral("actionAdd"));
        actionDelete = new QAction(MainWindow);
        actionDelete->setObjectName(QStringLiteral("actionDelete"));
        actionChoose = new QAction(MainWindow);
        actionChoose->setObjectName(QStringLiteral("actionChoose"));
        actionParameters = new QAction(MainWindow);
        actionParameters->setObjectName(QStringLiteral("actionParameters"));
        actionStrategy_control = new QAction(MainWindow);
        actionStrategy_control->setObjectName(QStringLiteral("actionStrategy_control"));
        actionSensors = new QAction(MainWindow);
        actionSensors->setObjectName(QStringLiteral("actionSensors"));
        actionActuators = new QAction(MainWindow);
        actionActuators->setObjectName(QStringLiteral("actionActuators"));
        actionSave_2 = new QAction(MainWindow);
        actionSave_2->setObjectName(QStringLiteral("actionSave_2"));
        actionSave_as = new QAction(MainWindow);
        actionSave_as->setObjectName(QStringLiteral("actionSave_as"));
        actionDelete_2 = new QAction(MainWindow);
        actionDelete_2->setObjectName(QStringLiteral("actionDelete_2"));
        actionPhysics = new QAction(MainWindow);
        actionPhysics->setObjectName(QStringLiteral("actionPhysics"));
        actionGravity = new QAction(MainWindow);
        actionGravity->setObjectName(QStringLiteral("actionGravity"));
        actionParameters_2 = new QAction(MainWindow);
        actionParameters_2->setObjectName(QStringLiteral("actionParameters_2"));
        actionSernsors = new QAction(MainWindow);
        actionSernsors->setObjectName(QStringLiteral("actionSernsors"));
        actionActuators_2 = new QAction(MainWindow);
        actionActuators_2->setObjectName(QStringLiteral("actionActuators_2"));
        actionControl_Strategy = new QAction(MainWindow);
        actionControl_Strategy->setObjectName(QStringLiteral("actionControl_Strategy"));
        actionNew_2 = new QAction(MainWindow);
        actionNew_2->setObjectName(QStringLiteral("actionNew_2"));
        actionRemove = new QAction(MainWindow);
        actionRemove->setObjectName(QStringLiteral("actionRemove"));
        actionSensors_2 = new QAction(MainWindow);
        actionSensors_2->setObjectName(QStringLiteral("actionSensors_2"));
        actionActuators_3 = new QAction(MainWindow);
        actionActuators_3->setObjectName(QStringLiteral("actionActuators_3"));
        actionControl_Strategy_2 = new QAction(MainWindow);
        actionControl_Strategy_2->setObjectName(QStringLiteral("actionControl_Strategy_2"));
        actionEdit_link = new QAction(MainWindow);
        actionEdit_link->setObjectName(QStringLiteral("actionEdit_link"));
        actionOint = new QAction(MainWindow);
        actionOint->setObjectName(QStringLiteral("actionOint"));
        actionPlugin = new QAction(MainWindow);
        actionPlugin->setObjectName(QStringLiteral("actionPlugin"));
        actionEdit = new QAction(MainWindow);
        actionEdit->setObjectName(QStringLiteral("actionEdit"));
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionAbout_ProVANT_Simulator = new QAction(MainWindow);
        actionAbout_ProVANT_Simulator->setObjectName(QStringLiteral("actionAbout_ProVANT_Simulator"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        treeWidget = new QTreeWidget(centralWidget);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        treeWidget->setGeometry(QRect(20, 30, 331, 421));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(720, 390, 121, 41));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(480, 10, 67, 17));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(750, 10, 67, 17));
        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(380, 30, 256, 192));
        graphicsView_2 = new QGraphicsView(centralWidget);
        graphicsView_2->setObjectName(QStringLiteral("graphicsView_2"));
        graphicsView_2->setGeometry(QRect(650, 30, 256, 192));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(400, 306, 291, 121));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 939, 25));
        menuSimulation = new QMenu(menuBar);
        menuSimulation->setObjectName(QStringLiteral("menuSimulation"));
        menuEdit = new QMenu(menuBar);
        menuEdit->setObjectName(QStringLiteral("menuEdit"));
        menuModel = new QMenu(menuEdit);
        menuModel->setObjectName(QStringLiteral("menuModel"));
        menuAbout = new QMenu(menuBar);
        menuAbout->setObjectName(QStringLiteral("menuAbout"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuSimulation->menuAction());
        menuBar->addAction(menuEdit->menuAction());
        menuBar->addAction(menuAbout->menuAction());
        menuSimulation->addAction(actionNew);
        menuSimulation->addAction(actionOpen);
        menuSimulation->addAction(actionSave);
        menuSimulation->addAction(actionExit);
        menuEdit->addAction(menuModel->menuAction());
        menuModel->addAction(actionNew_2);
        menuAbout->addAction(actionAbout_ProVANT_Simulator);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        actionNew->setText(QApplication::translate("MainWindow", "New", 0));
        actionSave->setText(QApplication::translate("MainWindow", "Save", 0));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0));
        actionAdd->setText(QApplication::translate("MainWindow", "New", 0));
        actionDelete->setText(QApplication::translate("MainWindow", "Delete", 0));
        actionChoose->setText(QApplication::translate("MainWindow", "World", 0));
        actionParameters->setText(QApplication::translate("MainWindow", "Parameters", 0));
        actionStrategy_control->setText(QApplication::translate("MainWindow", "Control Strategy", 0));
        actionSensors->setText(QApplication::translate("MainWindow", "Sensors", 0));
        actionActuators->setText(QApplication::translate("MainWindow", "Actuators", 0));
        actionSave_2->setText(QApplication::translate("MainWindow", "Save", 0));
        actionSave_as->setText(QApplication::translate("MainWindow", "Save as", 0));
        actionDelete_2->setText(QApplication::translate("MainWindow", "Delete", 0));
        actionPhysics->setText(QApplication::translate("MainWindow", "Physics", 0));
        actionGravity->setText(QApplication::translate("MainWindow", "Gravity", 0));
        actionParameters_2->setText(QApplication::translate("MainWindow", "Parameters", 0));
        actionSernsors->setText(QApplication::translate("MainWindow", "Sensors", 0));
        actionActuators_2->setText(QApplication::translate("MainWindow", "Actuators", 0));
        actionControl_Strategy->setText(QApplication::translate("MainWindow", "Control Strategy", 0));
        actionNew_2->setText(QApplication::translate("MainWindow", "New", 0));
        actionRemove->setText(QApplication::translate("MainWindow", "Remove", 0));
        actionSensors_2->setText(QApplication::translate("MainWindow", "Sensors", 0));
        actionActuators_3->setText(QApplication::translate("MainWindow", "Actuators", 0));
        actionControl_Strategy_2->setText(QApplication::translate("MainWindow", "Control Strategy", 0));
        actionEdit_link->setText(QApplication::translate("MainWindow", "Link", 0));
        actionOint->setText(QApplication::translate("MainWindow", "Joint", 0));
        actionPlugin->setText(QApplication::translate("MainWindow", "Plugin", 0));
        actionEdit->setText(QApplication::translate("MainWindow", "Edit", 0));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", 0));
        actionAbout_ProVANT_Simulator->setText(QApplication::translate("MainWindow", "About ProVANT Simulator", 0));
        QTreeWidgetItem *___qtreewidgetitem = treeWidget->headerItem();
        ___qtreewidgetitem->setText(1, QApplication::translate("MainWindow", "Value", 0));
        ___qtreewidgetitem->setText(0, QApplication::translate("MainWindow", "Item", 0));
        pushButton->setText(QApplication::translate("MainWindow", "Startup Gazebo", 0));
        label->setText(QApplication::translate("MainWindow", "Model", 0));
        label_2->setText(QApplication::translate("MainWindow", "World", 0));
        label_3->setText(QString());
        menuSimulation->setTitle(QApplication::translate("MainWindow", "Simulation", 0));
        menuEdit->setTitle(QApplication::translate("MainWindow", "Edit", 0));
        menuModel->setTitle(QApplication::translate("MainWindow", "Model", 0));
        menuAbout->setTitle(QApplication::translate("MainWindow", "About", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
