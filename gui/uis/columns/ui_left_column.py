# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'left_columnJmijyN.ui'
##
## Created by: Qt User Interface Compiler version 6.4.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from qt_core import *

class Ui_LeftColumn(object):
    def setupUi(self, LeftColumn):
        if not LeftColumn.objectName():
            LeftColumn.setObjectName(u"LeftColumn")
        LeftColumn.resize(240, 600)
        self.main_pages_layout = QVBoxLayout(LeftColumn)
        self.main_pages_layout.setSpacing(0)
        self.main_pages_layout.setObjectName(u"main_pages_layout")
        self.main_pages_layout.setContentsMargins(5, 5, 5, 5)
        self.menus = QStackedWidget(LeftColumn)
        self.menus.setObjectName(u"menus")
        self.menu_2 = QWidget()
        self.menu_2.setObjectName(u"menu_2")
        self.verticalLayout = QVBoxLayout(self.menu_2)
        self.verticalLayout.setSpacing(5)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(5, 5, 5, 5)
        self.label_1 = QLabel(self.menu_2)
        self.label_1.setObjectName(u"label_1")
        font = QFont()
        font.setPointSize(16)
        self.label_1.setFont(font)
        self.label_1.setStyleSheet(u"font-size: 16pt")
        self.label_1.setAlignment(Qt.AlignCenter)

        self.verticalLayout.addWidget(self.label_1)

        self.menus.addWidget(self.menu_2)
        self.menu_1 = QWidget()
        self.menu_1.setObjectName(u"menu_1")
        self.verticalLayout_2 = QVBoxLayout(self.menu_1)
        self.verticalLayout_2.setSpacing(5)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(5, 5, 5, 5)
        self.frame_btn1 = QFrame(self.menu_1)
        self.frame_btn1.setObjectName(u"frame_btn1")
        self.frame_btn1.setMinimumSize(QSize(0, 40))
        self.frame_btn1.setMaximumSize(QSize(16777215, 40))
        self.frame_btn1.setFrameShape(QFrame.NoFrame)
        self.frame_btn1.setFrameShadow(QFrame.Raised)
        self.btn1_layout = QVBoxLayout(self.frame_btn1)
        self.btn1_layout.setSpacing(0)
        self.btn1_layout.setObjectName(u"btn1_layout")
        self.btn1_layout.setContentsMargins(0, 0, 0, 0)

        self.verticalLayout_2.addWidget(self.frame_btn1)

        self.frame_btn2 = QFrame(self.menu_1)
        self.frame_btn2.setObjectName(u"frame_btn2")
        self.frame_btn2.setMinimumSize(QSize(0, 40))
        self.frame_btn2.setMaximumSize(QSize(16777215, 40))
        self.frame_btn2.setFrameShape(QFrame.NoFrame)
        self.frame_btn2.setFrameShadow(QFrame.Raised)
        self.btn2_layout = QVBoxLayout(self.frame_btn2)
        self.btn2_layout.setSpacing(0)
        self.btn2_layout.setObjectName(u"btn2_layout")
        self.btn2_layout.setContentsMargins(0, 0, 0, 0)

        self.verticalLayout_2.addWidget(self.frame_btn2)

        self.label_2 = QLabel(self.menu_1)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setFont(font)
        self.label_2.setStyleSheet(u"font-size: 16pt")
        self.label_2.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.label_2)

        self.menus.addWidget(self.menu_1)

        self.main_pages_layout.addWidget(self.menus)


        self.retranslateUi(LeftColumn)

        self.menus.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(LeftColumn)
    # setupUi

    def retranslateUi(self, LeftColumn):
        LeftColumn.setWindowTitle(QCoreApplication.translate("LeftColumn", u"Form", None))
        self.label_1.setText(QCoreApplication.translate("LeftColumn", u"Menu 2 - Left Menu", None))
        self.label_2.setText(QCoreApplication.translate("LeftColumn", u"Menu 1 - Left Menu", None))
    # retranslateUi

