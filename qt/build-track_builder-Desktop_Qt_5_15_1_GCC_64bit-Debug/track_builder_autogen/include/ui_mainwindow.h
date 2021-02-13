/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *connectCamerasBtn;
    QPushButton *captureBtn;
    QGraphicsView *leftView;
    QGraphicsView *rightView;
    QGraphicsView *filterView;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1145, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(1050, 0, 91, 80));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        connectCamerasBtn = new QPushButton(verticalLayoutWidget);
        connectCamerasBtn->setObjectName(QString::fromUtf8("connectCamerasBtn"));

        verticalLayout->addWidget(connectCamerasBtn);

        captureBtn = new QPushButton(verticalLayoutWidget);
        captureBtn->setObjectName(QString::fromUtf8("captureBtn"));

        verticalLayout->addWidget(captureBtn);

        leftView = new QGraphicsView(centralwidget);
        leftView->setObjectName(QString::fromUtf8("leftView"));
        leftView->setGeometry(QRect(10, 0, 512, 410));
        rightView = new QGraphicsView(centralwidget);
        rightView->setObjectName(QString::fromUtf8("rightView"));
        rightView->setGeometry(QRect(530, 0, 512, 410));
        filterView = new QGraphicsView(centralwidget);
        filterView->setObjectName(QString::fromUtf8("filterView"));
        filterView->setGeometry(QRect(10, 420, 171, 121));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1145, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Track Builder", nullptr));
        connectCamerasBtn->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        captureBtn->setText(QCoreApplication::translate("MainWindow", "Capture", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
