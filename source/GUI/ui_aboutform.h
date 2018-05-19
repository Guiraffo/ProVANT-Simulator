/********************************************************************************
** Form generated from reading UI file 'aboutform.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUTFORM_H
#define UI_ABOUTFORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AboutForm
{
public:
    QTextEdit *textEdit;
    QLabel *label;

    void setupUi(QWidget *AboutForm)
    {
        if (AboutForm->objectName().isEmpty())
            AboutForm->setObjectName(QStringLiteral("AboutForm"));
        AboutForm->resize(400, 300);
        textEdit = new QTextEdit(AboutForm);
        textEdit->setObjectName(QStringLiteral("textEdit"));
        textEdit->setGeometry(QRect(0, 0, 401, 141));
        label = new QLabel(AboutForm);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 150, 67, 17));

        retranslateUi(AboutForm);

        QMetaObject::connectSlotsByName(AboutForm);
    } // setupUi

    void retranslateUi(QWidget *AboutForm)
    {
        AboutForm->setWindowTitle(QApplication::translate("AboutForm", "Form", 0));
        textEdit->setHtml(QApplication::translate("AboutForm", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><a name=\"tw-target\"></a><span style=\" font-family:'monospace';\">T</span><span style=\" font-family:'monospace';\">his work was produced under the ProVANT project, a research project resulting from the partnership between the Federal University of Minas Gerais, Federal University of Santa Catarina and University of Seville and with support CNPq, CAPES, FAPEMIG and InSAC</span></p></body></html>", 0));
        label->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class AboutForm: public Ui_AboutForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUTFORM_H
