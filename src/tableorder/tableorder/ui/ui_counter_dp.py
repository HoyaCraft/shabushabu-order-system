# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'counter_dp.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1280, 720)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabMain = QWidget()
        self.tabMain.setObjectName(u"tabMain")
        self.batteryFrame = QFrame(self.tabMain)
        self.batteryFrame.setObjectName(u"batteryFrame")
        self.batteryFrame.setGeometry(QRect(400, 20, 100, 100))
        self.listWidgetWaiting = QListWidget(self.tabMain)
        self.listWidgetWaiting.setObjectName(u"listWidgetWaiting")
        self.listWidgetWaiting.setGeometry(QRect(20, 60, 200, 180))
        self.listWidgetWaiting.setLayoutDirection(Qt.LeftToRight)
        self.listWidgetWaiting.setStyleSheet(u"")
        self.listWidgetCooking = QListWidget(self.tabMain)
        self.listWidgetCooking.setObjectName(u"listWidgetCooking")
        self.listWidgetCooking.setGeometry(QRect(20, 300, 200, 180))
        self.listWidgetCooking.setStyleSheet(u"")
        self.listWidgetStock = QListWidget(self.tabMain)
        self.listWidgetStock.setObjectName(u"listWidgetStock")
        self.listWidgetStock.setGeometry(QRect(400, 140, 100, 200))
        self.pushButtonCheckWaiting = QPushButton(self.tabMain)
        self.pushButtonCheckWaiting.setObjectName(u"pushButtonCheckWaiting")
        self.pushButtonCheckWaiting.setGeometry(QRect(20, 20, 90, 30))
        self.pushButtonOk = QPushButton(self.tabMain)
        self.pushButtonOk.setObjectName(u"pushButtonOk")
        self.pushButtonOk.setGeometry(QRect(120, 20, 60, 30))
        self.pushButtonOk.setVisible(False)
        self.pushButtonSorry = QPushButton(self.tabMain)
        self.pushButtonSorry.setObjectName(u"pushButtonSorry")
        self.pushButtonSorry.setGeometry(QRect(190, 20, 60, 30))
        self.pushButtonSorry.setVisible(False)
        self.tabWidget.addTab(self.tabMain, "")
        self.tabEdit = QWidget()
        self.tabEdit.setObjectName(u"tabEdit")
        self.verticalLayoutEdit = QVBoxLayout(self.tabEdit)
        self.verticalLayoutEdit.setObjectName(u"verticalLayoutEdit")
        self.tableWidgetMenu = QTableWidget(self.tabEdit)
        if (self.tableWidgetMenu.columnCount() < 4):
            self.tableWidgetMenu.setColumnCount(4)
        __qtablewidgetitem = QTableWidgetItem()
        self.tableWidgetMenu.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.tableWidgetMenu.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.tableWidgetMenu.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.tableWidgetMenu.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        if (self.tableWidgetMenu.rowCount() < 5):
            self.tableWidgetMenu.setRowCount(5)
        self.tableWidgetMenu.setObjectName(u"tableWidgetMenu")
        self.tableWidgetMenu.setRowCount(5)
        self.tableWidgetMenu.setColumnCount(4)

        self.verticalLayoutEdit.addWidget(self.tableWidgetMenu)

        self.horizontalLayoutSaveReset = QHBoxLayout()
        self.horizontalLayoutSaveReset.setObjectName(u"horizontalLayoutSaveReset")
        self.pushButtonSave = QPushButton(self.tabEdit)
        self.pushButtonSave.setObjectName(u"pushButtonSave")

        self.horizontalLayoutSaveReset.addWidget(self.pushButtonSave)

        self.pushButtonReset = QPushButton(self.tabEdit)
        self.pushButtonReset.setObjectName(u"pushButtonReset")

        self.horizontalLayoutSaveReset.addWidget(self.pushButtonReset)


        self.verticalLayoutEdit.addLayout(self.horizontalLayoutSaveReset)

        self.tabWidget.addTab(self.tabEdit, "")

        self.verticalLayout.addWidget(self.tabWidget)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1280, 28))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Counter Display", None))
        self.pushButtonCheckWaiting.setText(QCoreApplication.translate("MainWindow", u"\uacb0\uc81c \ub300\uae30 \uc911", None))
        self.pushButtonOk.setText(QCoreApplication.translate("MainWindow", u"OK", None))
        self.pushButtonSorry.setText(QCoreApplication.translate("MainWindow", u"SORRY", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabMain), QCoreApplication.translate("MainWindow", u"Main", None))
        ___qtablewidgetitem = self.tableWidgetMenu.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"\uba54\ub274\uba85", None));
        ___qtablewidgetitem1 = self.tableWidgetMenu.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"\uac00\uaca9", None));
        ___qtablewidgetitem2 = self.tableWidgetMenu.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"\uc7ac\uace0", None));
        ___qtablewidgetitem3 = self.tableWidgetMenu.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"\uc218\uc815", None));
        self.pushButtonSave.setText(QCoreApplication.translate("MainWindow", u"Save", None))
        self.pushButtonReset.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabEdit), QCoreApplication.translate("MainWindow", u"Edit", None))
    # retranslateUi

