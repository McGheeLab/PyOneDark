# ///////////////////////////////////////////////////////////////
# Imports
# ///////////////////////////////////////////////////////////////
from qt_core import *

# ///////////////////////////////////////////////////////////////
# Main UI Class
# ///////////////////////////////////////////////////////////////
class Ui_MainPages(object):

    def setupUi(self, MainPages):
        # Set object name and initial window size
        if not MainPages.objectName():
            MainPages.setObjectName("MainPages")
        MainPages.resize(860, 600)

        # ///////////////////////////////////////////////////////////////
        # Main Layout for the Window
        # ///////////////////////////////////////////////////////////////
        self.main_pages_layout = QVBoxLayout(MainPages)
        self.main_pages_layout.setSpacing(0)
        self.main_pages_layout.setObjectName("main_pages_layout")
        self.main_pages_layout.setContentsMargins(5, 5, 5, 5)

        # ///////////////////////////////////////////////////////////////
        # QStackedWidget to hold multiple pages
        # ///////////////////////////////////////////////////////////////
        self.pages = QStackedWidget(MainPages)
        self.pages.setObjectName("pages")

        # ///////////////////////////////////////////////////////////////
        # Page 1 - Welcome Page
        # ///////////////////////////////////////////////////////////////
        self.page_1 = QWidget()
        self.page_1.setObjectName("page_1")
        self.page_1.setStyleSheet("font-size: 14pt")
        self.page_1_layout = QVBoxLayout(self.page_1)
        self.page_1_layout.setSpacing(5)
        self.page_1_layout.setObjectName("page_1_layout")
        self.page_1_layout.setContentsMargins(5, 5, 5, 5)

        # Frame to hold welcome elements (logo and label)
        self.welcome_base = QFrame(self.page_1)
        self.welcome_base.setObjectName("welcome_base")
        self.welcome_base.setMinimumSize(QSize(300, 150))
        self.welcome_base.setMaximumSize(QSize(300, 150))
        self.welcome_base.setFrameShape(QFrame.NoFrame)
        self.welcome_base.setFrameShadow(QFrame.Raised)

        # Layout for centering welcome elements inside welcome_base
        self.center_page_layout = QVBoxLayout(self.welcome_base)
        self.center_page_layout.setSpacing(10)
        self.center_page_layout.setObjectName("center_page_layout")
        self.center_page_layout.setContentsMargins(0, 0, 0, 0)

        # Frame for displaying the logo
        self.logo = QFrame(self.welcome_base)
        self.logo.setObjectName("logo")
        self.logo.setMinimumSize(QSize(300, 120))
        self.logo.setMaximumSize(QSize(300, 120))
        self.logo.setFrameShape(QFrame.NoFrame)
        self.logo.setFrameShadow(QFrame.Raised)
        self.logo_layout = QVBoxLayout(self.logo)
        self.logo_layout.setSpacing(0)
        self.logo_layout.setObjectName("logo_layout")
        self.logo_layout.setContentsMargins(0, 0, 0, 0)

        # Add the logo frame (placeholder) to the center layout
        self.center_page_layout.addWidget(self.logo)

        # Label for welcome text
        self.label = QLabel(self.welcome_base)
        self.label.setObjectName("label")
        self.label.setAlignment(Qt.AlignCenter)
        self.center_page_layout.addWidget(self.label)

        # Center the welcome_base frame on page 1
        self.page_1_layout.addWidget(self.welcome_base, 0, Qt.AlignHCenter)

        # Add page 1 to the pages widget (stacked widget)
        self.pages.addWidget(self.page_1)

        # ///////////////////////////////////////////////////////////////
        # Page 2 - Custom Widgets Page
        # ///////////////////////////////////////////////////////////////
        self.page_2 = QWidget()
        self.page_2.setObjectName("page_2")
        self.page_2_layout = QVBoxLayout(self.page_2)
        self.page_2_layout.setSpacing(5)
        self.page_2_layout.setObjectName("page_2_layout")
        self.page_2_layout.setContentsMargins(5, 5, 5, 5)

        # Scroll area to allow scrolling of content
        self.scroll_area = QScrollArea(self.page_2)
        self.scroll_area.setObjectName("scroll_area")
        self.scroll_area.setStyleSheet("background: transparent;")
        self.scroll_area.setFrameShape(QFrame.NoFrame)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setWidgetResizable(True)

        # Widget that contains all scrollable content
        self.contents = QWidget()
        self.contents.setObjectName("contents")
        self.contents.setGeometry(QRect(0, 0, 840, 580))
        self.contents.setStyleSheet("background: transparent;")
        self.verticalLayout = QVBoxLayout(self.contents)
        self.verticalLayout.setSpacing(15)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(5, 5, 5, 5)

        # Title label for the page
        self.title_label = QLabel(self.contents)
        self.title_label.setObjectName("title_label")
        self.title_label.setMaximumSize(QSize(16777215, 40))
        font = QFont()
        font.setPointSize(16)
        self.title_label.setFont(font)
        self.title_label.setStyleSheet("font-size: 16pt")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.verticalLayout.addWidget(self.title_label)

        # Description label for further details
        self.description_label = QLabel(self.contents)
        self.description_label.setObjectName("description_label")
        self.description_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.description_label.setWordWrap(True)
        self.verticalLayout.addWidget(self.description_label)

        # Layouts for custom widgets organization (rows)
        self.row_1_layout = QHBoxLayout()
        self.row_1_layout.setObjectName("row_1_layout")
        self.verticalLayout.addLayout(self.row_1_layout)

        self.row_2_layout = QHBoxLayout()
        self.row_2_layout.setObjectName("row_2_layout")
        self.verticalLayout.addLayout(self.row_2_layout)

        self.row_3_layout = QHBoxLayout()
        self.row_3_layout.setObjectName("row_3_layout")
        self.verticalLayout.addLayout(self.row_3_layout)

        self.row_4_layout = QVBoxLayout()
        self.row_4_layout.setObjectName("row_4_layout")
        self.verticalLayout.addLayout(self.row_4_layout)

        self.row_5_layout = QVBoxLayout()
        self.row_5_layout.setObjectName("row_5_layout")
        self.verticalLayout.addLayout(self.row_5_layout)

        # Set the contents widget inside the scroll area
        self.scroll_area.setWidget(self.contents)
        self.page_2_layout.addWidget(self.scroll_area)

        # Add page 2 to the stacked pages widget
        self.pages.addWidget(self.page_2)

        # ///////////////////////////////////////////////////////////////
        # Page 3 - Empty Page (Placeholder)
        # ///////////////////////////////////////////////////////////////
        self.page_3 = QWidget()
        self.page_3.setObjectName("page_3")
        self.page_3.setStyleSheet("QFrame { font-size: 16pt; }")
        self.page_3_layout = QVBoxLayout(self.page_3)
        self.page_3_layout.setObjectName("page_3_layout")

        # Label to display that this page is empty
        self.empty_page_label = QLabel(self.page_3)
        self.empty_page_label.setObjectName("empty_page_label")
        self.empty_page_label.setFont(font)
        self.empty_page_label.setAlignment(Qt.AlignCenter)
        self.page_3_layout.addWidget(self.empty_page_label)

        self.pages.addWidget(self.page_3)

        # ///////////////////////////////////////////////////////////////
        # Final Assembly: Add the QStackedWidget to the Main Layout
        # ///////////////////////////////////////////////////////////////
        self.main_pages_layout.addWidget(self.pages)

        # Set the initial page index (0: Page 1)
        self.pages.setCurrentIndex(0)

        # Auto-connect signals and slots
        QMetaObject.connectSlotsByName(MainPages)

        # Translate UI texts
        self.retranslateUi(MainPages)

    def retranslateUi(self, MainPages):
        # Set window title and label texts; these can be localized
        MainPages.setWindowTitle(QCoreApplication.translate("MainPages", "Form", None))
        self.label.setText(QCoreApplication.translate("MainPages", "Welcome To PyOneDark GUI", None))
        self.title_label.setText(QCoreApplication.translate("MainPages", "Custom Widgets Page", None))
        self.description_label.setText(
            QCoreApplication.translate(
                "MainPages",
                "Here will be all the custom widgets, they will be added over time on this page.\n"
                "I will try to always record a new tutorial when adding a new Widget and updating the project on Patreon before launching on GitHub and GitHub after the public release.",
                None
            )
        )
        self.empty_page_label.setText(QCoreApplication.translate("MainPages", "Empty Page", None))
