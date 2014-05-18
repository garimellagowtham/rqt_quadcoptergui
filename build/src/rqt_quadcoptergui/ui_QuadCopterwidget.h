/********************************************************************************
** Form generated from reading UI file 'QuadCopterwidget.ui'
**
** Created: Thu May 8 14:09:18 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QUADCOPTERWIDGET_H
#define UI_QUADCOPTERWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QTextBrowser>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QuadCopterwidget
{
public:
    QPushButton *Landbutton;
    QTextBrowser *textBrowser;
    QPushButton *Takeoffbutton;
    QLabel *label;
    QCheckBox *imucheckbox;
    QCheckBox *thrust_estcheckbox;
    QPushButton *Disarmbutton;
    QCheckBox *enable_controller;
    QCheckBox *perturb_checkbox;
    QCheckBox *integrator_checkbox;
    QCheckBox *log_checkbox;

    void setupUi(QWidget *QuadCopterwidget)
    {
        if (QuadCopterwidget->objectName().isEmpty())
            QuadCopterwidget->setObjectName(QString::fromUtf8("QuadCopterwidget"));
        QuadCopterwidget->resize(323, 513);
        Landbutton = new QPushButton(QuadCopterwidget);
        Landbutton->setObjectName(QString::fromUtf8("Landbutton"));
        Landbutton->setGeometry(QRect(120, 450, 93, 24));
        textBrowser = new QTextBrowser(QuadCopterwidget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(10, 100, 311, 311));
        Takeoffbutton = new QPushButton(QuadCopterwidget);
        Takeoffbutton->setObjectName(QString::fromUtf8("Takeoffbutton"));
        Takeoffbutton->setGeometry(QRect(20, 450, 93, 24));
        label = new QLabel(QuadCopterwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(100, 20, 81, 31));
        imucheckbox = new QCheckBox(QuadCopterwidget);
        imucheckbox->setObjectName(QString::fromUtf8("imucheckbox"));
        imucheckbox->setGeometry(QRect(120, 80, 101, 21));
        thrust_estcheckbox = new QCheckBox(QuadCopterwidget);
        thrust_estcheckbox->setObjectName(QString::fromUtf8("thrust_estcheckbox"));
        thrust_estcheckbox->setGeometry(QRect(220, 80, 101, 21));
        Disarmbutton = new QPushButton(QuadCopterwidget);
        Disarmbutton->setObjectName(QString::fromUtf8("Disarmbutton"));
        Disarmbutton->setGeometry(QRect(220, 450, 93, 24));
        enable_controller = new QCheckBox(QuadCopterwidget);
        enable_controller->setObjectName(QString::fromUtf8("enable_controller"));
        enable_controller->setGeometry(QRect(20, 80, 101, 21));
        perturb_checkbox = new QCheckBox(QuadCopterwidget);
        perturb_checkbox->setObjectName(QString::fromUtf8("perturb_checkbox"));
        perturb_checkbox->setGeometry(QRect(220, 50, 101, 21));
        integrator_checkbox = new QCheckBox(QuadCopterwidget);
        integrator_checkbox->setObjectName(QString::fromUtf8("integrator_checkbox"));
        integrator_checkbox->setGeometry(QRect(120, 50, 101, 21));
        log_checkbox = new QCheckBox(QuadCopterwidget);
        log_checkbox->setObjectName(QString::fromUtf8("log_checkbox"));
        log_checkbox->setGeometry(QRect(20, 50, 101, 21));

        retranslateUi(QuadCopterwidget);

        QMetaObject::connectSlotsByName(QuadCopterwidget);
    } // setupUi

    void retranslateUi(QWidget *QuadCopterwidget)
    {
        QuadCopterwidget->setWindowTitle(QApplication::translate("QuadCopterwidget", "Form", 0, QApplication::UnicodeUTF8));
        Landbutton->setText(QApplication::translate("QuadCopterwidget", "Land", 0, QApplication::UnicodeUTF8));
        Takeoffbutton->setText(QApplication::translate("QuadCopterwidget", "Takeoff", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("QuadCopterwidget", "<html><head/><body><p>MAV_Status</p></body></html>", 0, QApplication::UnicodeUTF8));
        imucheckbox->setText(QApplication::translate("QuadCopterwidget", "imu_recalib", 0, QApplication::UnicodeUTF8));
        thrust_estcheckbox->setText(QApplication::translate("QuadCopterwidget", "thrust_est", 0, QApplication::UnicodeUTF8));
        Disarmbutton->setText(QApplication::translate("QuadCopterwidget", "Disarm", 0, QApplication::UnicodeUTF8));
        enable_controller->setText(QApplication::translate("QuadCopterwidget", "enable_ctrl", 0, QApplication::UnicodeUTF8));
        perturb_checkbox->setText(QApplication::translate("QuadCopterwidget", "enable_pert", 0, QApplication::UnicodeUTF8));
        integrator_checkbox->setText(QApplication::translate("QuadCopterwidget", "disable_int", 0, QApplication::UnicodeUTF8));
        log_checkbox->setText(QApplication::translate("QuadCopterwidget", "enable_log", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class QuadCopterwidget: public Ui_QuadCopterwidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QUADCOPTERWIDGET_H
