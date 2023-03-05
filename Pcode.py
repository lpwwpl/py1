import sys
import os
import logging

from PyQt5 import QtCore, QtGui, QtWidgets


from Extensions.UseData import UseData
from Extensions.About import About
from Extensions.Settings.SettingsWidget import SettingsWidget
from Extensions.Projects.Projects import Projects
from Extensions.BusyWidget import BusyWidget
from Extensions import StyleSheet
from Extensions.Start import Start
from Extensions.StackSwitcher import StackSwitcher

# env = Environment()
# time.sleep(2)
#
# print("[INFO] Start manual control!")
# env.start_manual_control()
class Pcode(QtWidgets.QWidget):

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)

        self.setWindowIcon(
            QtGui.QIcon(os.path.join("Resources", "images", "Icon")))
        self.setWindowTitle("Ascode - Loading...")

        screen = QtWidgets.QDesktopWidget().screenGeometry()
        self.resize(screen.width() - 200, screen.height() - 200)
        size = self.geometry()
        self.move( int((screen.width() - size.width()) / 2), int((
            screen.height() - size.height()) / 2))
        self.lastWindowGeometry = self.geometry()

        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setSpacing(0)
        mainLayout.setContentsMargins(0,0,0,0) #.setContentsMargins(0,0,0,0)
        self.setLayout(mainLayout)

        self.useData = UseData()

        logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s',
                            filename=self.useData.appPathDict["logfile"], level=logging.DEBUG)
        if sys.version_info.major < 3:
            logging.error("This application requires Python 3")
            sys.exit(1)

        self.busyWidget = BusyWidget(app, self.useData, self)

        if self.useData.SETTINGS["UI"] == "Custom":
            app.setStyleSheet(StyleSheet.globalStyle)

        self.projectWindowStack = QtWidgets.QStackedWidget()

        self.projectTitleBox = QtWidgets.QComboBox()
        self.projectTitleBox.setMinimumWidth(180)
        self.projectTitleBox.setStyleSheet(StyleSheet.projectTitleBoxStyle)
        self.projectTitleBox.setItemDelegate(QtWidgets.QStyledItemDelegate())
        self.projectTitleBox.currentIndexChanged.connect(self.projectChanged)
        self.projectTitleBox.activated.connect(self.projectChanged)

        self.settingsWidget = SettingsWidget(self.useData, app,
                                             self.projectWindowStack,  self)

        startWindow = Start(self.useData, self)
        self.addProject(startWindow, "Start",
                        "Start", os.path.join("Resources", "images", "flag-green"))

        self.projects = Projects(self.useData, self.busyWidget, self.settingsWidget, app,
                                 self.projectWindowStack, self.projectTitleBox, self)

        self.createActions()

        hbox = QtWidgets.QHBoxLayout()
        hbox.setContentsMargins(5, 3, 5, 3)
        mainLayout.addLayout(hbox)

        # hbox.addStretch(1)

        self.pagesStack = QtWidgets.QStackedWidget()
        mainLayout.addWidget(self.pagesStack)

        self.projectSwitcher = StackSwitcher(self.pagesStack)
        self.projectSwitcher.setStyleSheet(StyleSheet.mainMenuStyle)
        hbox.addWidget(self.projectSwitcher)

        self.addPage(self.projectWindowStack, "EDITOR", QtGui.QIcon(
            os.path.join("Resources", "images", "hire-me")))

        self.projectSwitcher.setDefault()

        hbox.addWidget(self.projectTitleBox)
        hbox.setSpacing(5)

        self.settingsButton = QtWidgets.QToolButton()
        self.settingsButton.setAutoRaise(True)
        self.settingsButton.setDefaultAction(self.settingsAct)
        hbox.addWidget(self.settingsButton)

        self.fullScreenButton = QtWidgets.QToolButton()
        self.fullScreenButton.setAutoRaise(True)
        self.fullScreenButton.setDefaultAction(self.showFullScreenAct)
        hbox.addWidget(self.fullScreenButton)

        self.aboutButton = QtWidgets.QToolButton()
        self.aboutButton.setAutoRaise(True)
        self.aboutButton.setDefaultAction(self.aboutAct)
        hbox.addWidget(self.aboutButton)

        hbox.addStretch(1)

        self.setKeymap()

        if self.useData.settings["firstRun"] == 'True':
            self.showMaximized()
        else:
            self.restoreUiState()

        self.useData.settings["running"] = 'True'
        self.useData.settings["firstRun"] = 'False'
        self.useData.saveSettings()

    def createActions(self):
        self.aboutAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "properties")),
            "About Ascode", self, statusTip="About Ascode",
            triggered=self.showAbout)

        self.showFullScreenAct = \
            QtWidgets.QAction(
                QtGui.QIcon(os.path.join("Resources", "images", "fullscreen")),
                "Fullscreen", self,
                statusTip="Fullscreen",
                          triggered=self.showFullScreenMode)

        self.settingsAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "config")),
            "Settings", self,
            statusTip="Settings", triggered=self.showSettings)

    def addPage(self, pageWidget, name, iconPath):
        self.projectSwitcher.addButton(name=name, icon=iconPath)
        self.pagesStack.addWidget(pageWidget)

    def loadProject(self, path, show=False, new=False):
        self.projects.loadProject(path, show, new)

    def newProject(self):
        self.projects.newProjectDialog.exec_()

    def showProject(self, path):
        if not os.path.exists(path):
            message = QtWidgets.QMessageBox.warning(
                self, "Open Project", "Project cannot be be found!")
        else:
            if path in self.useData.OPENED_PROJECTS:
                for i in range(self.projectWindowStack.count() - 1):
                    window = self.projectWindowStack.widget(i)
                    p_path = window.projectPathDict["root"]
                    if os.path.samefile(path, p_path):
                        self.projectTitleBox.setCurrentIndex(i)
                        return True
        return False

    def addProject(self, window, name, type='Project', iconPath=None):
        self.projectWindowStack.insertWidget(0, window)
        if type == 'Project':
            self.projectTitleBox.insertItem(0, QtGui.QIcon(
                os.path.join("Resources", "images", "project")), name, [window, type])
        else:
            self.projectTitleBox.insertItem(0, QtGui.QIcon(
                iconPath), name, [window, type])

    def projectChanged(self, index):
        data = self.projectTitleBox.itemData(index)
        window = data[0]
        windowType = data[1]
        if windowType == "Start":
            self.setWindowTitle("Ascode - Start")
        elif windowType == "Project":
            title = window.editorTabWidget.getEditorData("filePath")
            self.updateWindowTitle(title)
        self.projectWindowStack.setCurrentWidget(window)

    def removeProject(self, window):
        for index in range(self.projectTitleBox.count() - 1):
            data = self.projectTitleBox.itemData(index)
            windowWidget = data[0]
            if windowWidget == window:
                self.projectWindowStack.removeWidget(window)
                self.projectTitleBox.removeItem(index)

    def updateWindowTitle(self, title):
        if title is None:
            title = "Ascode - " + "Unsaved"
        else:
            window = self.projectTitleBox.itemData(
                self.projectTitleBox.currentIndex())[0]
            if title.startswith(window.projectPathDict["sourcedir"]):
                src_dir = window.projectPathDict["sourcedir"]
                n = title.partition(src_dir)[-1]
                title = 'Ascode - ' + n
            else:
                title = "Ascode - " + title
        self.setWindowTitle(title)

    def showAbout(self):
        aboutPane = About(self)
        aboutPane.exec_()

    def showSettings(self):
        self.settingsWidget.show()

    def showFullScreenMode(self):
        if self.isFullScreen():
            self.showNormal()
            self.setGeometry(self.lastWindowGeometry)
        else:
            # get current size ahd show Fullscreen
            # so we can later restore to proper position
            self.lastWindowGeometry = self.geometry()
            self.showFullScreen()

    def saveUiState(self):
        settings = QtCore.QSettings("Clean Code Inc.", "Pcode")
        settings.beginGroup("MainWindow")
        settings.setValue("geometry", self.geometry())
        settings.setValue("snippetsMainsplitter",
                          self.settingsWidget.snippetEditor.mainSplitter.saveState())
        settings.setValue("windowMaximized", self.isMaximized())
        settings.endGroup()

    def restoreUiState(self):
        settings = QtCore.QSettings("Clean Code Inc.", "Pcode")
        settings.beginGroup("MainWindow")
        if settings.value("windowMaximized", True, type=bool):
            self.showMaximized()
        else:
            self.setGeometry(settings.value("geometry"))
            self.show()
        self.settingsWidget.snippetEditor.mainSplitter.restoreState(settings.value("snippetsMainsplitter"))
        settings.endGroup()

    def closeEvent(self, event):
        for i in range(self.projectWindowStack.count() - 1):
            window = self.projectWindowStack.widget(i)
            closed = window.closeWindow()
            if not closed:
                self.projectTitleBox.setCurrentIndex(i)
                event.ignore()
                return
            else:
                pass
        self.saveUiState()
        self.useData.saveUseData()
        app.closeAllWindows()

        event.accept()

    def setKeymap(self):
        shortcuts = self.useData.CUSTOM_SHORTCUTS

        self.shortFullscreen = QtWidgets.QShortcut(
            shortcuts["Ide"]["Fullscreen"], self)
        self.shortFullscreen.activated.connect(self.showFullScreenMode)

app = QtWidgets.QApplication(sys.argv)

splash = QtWidgets.QSplashScreen(
    QtGui.QPixmap(os.path.join("Resources", "images", "splash")))
splash.show()

main = Pcode()

splash.finish(main)

sys.exit(app.exec_())
