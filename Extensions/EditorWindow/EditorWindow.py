import os
import re
import sys
import traceback
import logging

from PyQt5 import QtCore, QtGui, QtXml, QtWidgets

from Extensions.FileExplorer import FileExplorer
from Extensions.BottomWidgets.FindInFiles import FindInFiles
from Extensions.Projects.ProjectManager.ProjectManager import ProjectManager
from Extensions.SearchWidget import SearchWidget
from Extensions.Outline.Outline import Outline
from Extensions.EditorTabWidget import EditorTabWidget
from Extensions.WritePad import WritePad
from Extensions.Favourites import Favourites
from Extensions.ExternalLauncher import ExternalLauncher
from Extensions.BottomWidgets.Assistant import Assistant
from Extensions.BottomWidgets.TasksWidget import Tasks
from Extensions.BottomWidgets.BookmarkWidget import BookmarkWidget
from Extensions.BottomWidgets.RunWidget import RunWidget
from Extensions.BottomWidgets.Messages import MessagesWidget
from Extensions.StackSwitcher import StackSwitcher
from Extensions import StyleSheet
from Extensions.EditorWindow.BuildStatusWidget import BuildStatusWidget
from Extensions.EditorWindow.VerticalSplitter import VerticalSplitter
from Extensions.BottomWidgets.Profiler import Profiler


class EditorWindow(QtWidgets.QWidget):

    def __init__(self, projectPathDict,  busyWidget,
                 colorScheme, useData, app, parent):
        QtWidgets.QWidget.__init__(self, parent)

        self.app = app
        self.useData = useData

        self.projects = parent
        self.colorScheme = colorScheme

        self.projectPathDict = projectPathDict
        self.loadProjectData()

        self.busyWidget = busyWidget
        self.buildStatusWidget = BuildStatusWidget(self.app, self.useData)

        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setContentsMargins(0,0,0,0)
        mainLayout.setSpacing(0)
        self.setLayout(mainLayout)

        self.standardToolbar = QtWidgets.QToolBar("Standard")
        self.standardToolbar.setMovable(False)
        self.standardToolbar.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.standardToolbar.setMaximumHeight(26)
        self.standardToolbar.setObjectName("StandardToolBar")
        mainLayout.addWidget(self.standardToolbar)

        widget = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout()
        vbox.setContentsMargins(0,0,0,0)
        vbox.setSpacing(0)
        widget.setLayout(vbox)

        self.vSplitter = VerticalSplitter()
        mainLayout.addWidget(self.vSplitter)

        self.hSplitter = QtWidgets.QSplitter()
        self.hSplitter.setObjectName("hSplitter")

        self.vSplitter.addWidget(self.hSplitter)

        self.bottomStack = QtWidgets.QStackedWidget()
        self.vSplitter.addWidget(self.bottomStack)

        ###############
        #self.hSplitter.addWidget(widget)

        self.bottomStackSwitcher = StackSwitcher(self.bottomStack)
        self.bottomStackSwitcher.setStyleSheet(StyleSheet.bottomSwitcherStyle)

        self.messagesWidget = MessagesWidget(
            self.bottomStackSwitcher, self.vSplitter)

        self.createActions()

        self.manageFavourites = Favourites(

            self.projectData['favourites'], self.messagesWidget, self)

        self.externalLauncher = ExternalLauncher(
            self.projectData["launchers"], self)

        self.writePad = WritePad(self.projectPathDict[
                                 "notes"], self.projectPathDict["name"], self)

        self.bookmarkToolbar = QtWidgets.QToolBar("Bookmarks")
        self.bookmarkToolbar.setMovable(False)
        self.bookmarkToolbar.setFloatable(False)
        self.bookmarkToolbar.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.bookmarkToolbar.setObjectName("Bookmarks")
        self.bookmarkToolbar.addSeparator()

        self.editorTabWidget = EditorTabWidget(
            self.useData, self.projectPathDict, self.projectData[
                "settings"], self.messagesWidget,
            self.colorScheme, self.busyWidget, self.bookmarkToolbar, self.app, self.manageFavourites,
            self.externalLauncher, self)
        vbox.addWidget(self.editorTabWidget)

        self.manageFavourites.openFile.connect(self.editorTabWidget.loadfile)

        self.editorTabWidget.updateRecentFilesList.connect(
            self.updateRecentFiles)
        self.editorTabWidget.updateLinesCount.connect(self.updateLineCount)
        self.editorTabWidget.updateEncodingLabel.connect(
            self.updateEncodingLabel)
        self.editorTabWidget.cursorPositionChanged.connect(
            self.showCursorPosition)

        self.searchWidget = SearchWidget(
            self.useData, self.editorTabWidget)
        vbox.addWidget(self.searchWidget)

        self.findInFiles = FindInFiles(
            self.useData, self.editorTabWidget, projectPathDict, self.bottomStackSwitcher)
        vbox.addWidget(self.findInFiles.dashboard)
        self.findInFiles.dashboard.hide()

        self.projectManager = ProjectManager(
            self.editorTabWidget, self.messagesWidget, projectPathDict, self.projectData[
                "settings"], self.useData, app,
            self.busyWidget, self.buildStatusWidget, self.projects)
        self.projectManager.projectView.fileActivated.connect(
            self.editorTabWidget.loadfile)

        self.outline = Outline(
            self.useData, self.editorTabWidget)

        self.sideSplitter = QtWidgets.QSplitter()
        self.sideSplitter.setObjectName("sidebarItem")
        self.sideSplitter.setOrientation(0)
        self.hSplitter.addWidget(self.sideSplitter)

        ###################
        self.hSplitter.addWidget(widget)

        self.sideSplitter.addWidget(self.outline)

        self.sideBottomTab = QtWidgets.QTabWidget()
        self.sideBottomTab.setObjectName("sideBottomTab")
        self.sideSplitter.addWidget(self.sideBottomTab)

        self.sideBottomTab.addTab(self.projectManager.projectView, QtGui.QIcon(
            os.path.join("Resources", "images", "tree")), "Project")

        self.fileExplorer = FileExplorer(
            self.useData, self.projectData['shortcuts'], self.messagesWidget, self.editorTabWidget)
        self.fileExplorer.fileActivated.connect(self.editorTabWidget.loadfile)
        # self.sideBottomTab.addTab(self.fileExplorer, QtGui.QIcon(
        #     os.path.join("Resources", "images", "tree")), "File System")

        # create menus
        self.mainMenu = QtWidgets.QMenu()
        self.mainMenu.addMenu(self.editorTabWidget.newFileMenu)
        self.mainMenu.addAction(self.editorTabWidget.openFileAct)
        self.mainMenu.addAction(self.editorTabWidget.saveAct)
        self.mainMenu.addAction(self.editorTabWidget.saveAllAct)
        self.mainMenu.addAction(self.editorTabWidget.saveAsAct)
        self.mainMenu.addAction(self.editorTabWidget.saveCopyAsAct)
        self.mainMenu.addAction(self.editorTabWidget.printAct)

        self.projectMenu = QtWidgets.QMenu("Project")
        if projectPathDict["type"] == "Desktop Application":
            self.projectMenu.addAction(self.buildAct)
        self.projectMenu.addSeparator()
        self.projectMenu.addAction(self.exportProjectAct)
        self.projectMenu.addAction(self.closeProjectAct)
        self.mainMenu.addMenu(self.projectMenu)

        self.mainMenu.addSeparator()
        self.mainMenu.addAction(self.gotoLineAct)
        self.mainMenu.addAction(self.viewSwitcherAct)
        self.mainMenu.addSeparator()
        self.mainMenu.addMenu(self.manageFavourites.favouritesMenu)
        self.recentFilesMenu = self.mainMenu.addMenu("Recent Files")
        self.recentFilesMenu.setIcon(
            QtGui.QIcon(os.path.join("Resources", "images", "history")))
        self.loadRecentFiles()
        self.mainMenu.addMenu(self.externalLauncher.launcherMenu)
        self.mainMenu.addSeparator()
        self.mainMenu.addAction(self.exitAct)

        self.createToolbars()

        # create StatusBar
        self.statusbar = QtWidgets.QStatusBar()

        self.statusbar.addPermanentWidget(self.buildStatusWidget)

        #*** Position
        self.cursorPositionButton = QtWidgets.QToolButton()
        self.cursorPositionButton.setAutoRaise(True)
        self.cursorPositionButton.clicked.connect(
            self.editorTabWidget.goToCursorPosition)
        self.statusbar.addPermanentWidget(self.cursorPositionButton)
        #*** lines
        self.linesLabel = QtWidgets.QLabel("Lines: 0")
        self.linesLabel.setMinimumWidth(50)
        self.statusbar.addPermanentWidget(self.linesLabel)
        #*** encoding
        self.encodingLabel = QtWidgets.QLabel("Coding: utf-8")
        self.statusbar.addPermanentWidget(self.encodingLabel)
        #*** uptime
        self.uptimeLabel = QtWidgets.QLabel()
        self.uptimeLabel.setText("Uptime: 0min")
        self.statusbar.addPermanentWidget(self.uptimeLabel)

        self.runWidget = RunWidget(
            self.bottomStackSwitcher, self.projectData[
                "settings"], self.useData,
            self.editorTabWidget, self.vSplitter,
            self.runProjectAct, self.stopRunAct, self.runFileAct)
        self.addBottomWidget(self.runWidget,
                             QtGui.QIcon(os.path.join("Resources", "images", "graphic-design")),  "Output")

        self.assistantWidget = Assistant(
            self.editorTabWidget, self.bottomStackSwitcher)
        self.addBottomWidget(self.assistantWidget,
                             QtGui.QIcon(os.path.join("Resources", "images", "flag")), "Alerts")

        bookmarkWidget = BookmarkWidget(
            self.editorTabWidget, self.bottomStackSwitcher)
        self.addBottomWidget(bookmarkWidget,
                             QtGui.QIcon(os.path.join("Resources", "images", "tag")), "Bookmarks")

        tasksWidget = Tasks(self.editorTabWidget, self.bottomStackSwitcher)
        self.addBottomWidget(tasksWidget,
                             QtGui.QIcon(os.path.join("Resources", "images", "issue")), "Tasks")

        self.addBottomWidget(self.messagesWidget,
                             QtGui.QIcon(os.path.join("Resources", "images", "speech_bubble")), "Messages")

        self.profiler = Profiler(self.useData, self.bottomStackSwitcher)
        self.addBottomWidget(self.profiler,
                             QtGui.QIcon(os.path.join("Resources", "images", "settings")), "Profiler")
        self.runWidget.loadProfile.connect(
            self.profiler.viewProfile)

        self.addBottomWidget(self.findInFiles,
                             QtGui.QIcon(os.path.join("Resources", "images", "attibutes")), "Find-in-Files")

        self.bottomStackSwitcher.setDefault()

        hbox = QtWidgets.QHBoxLayout()
        hbox.setContentsMargins(0,0,0,0)
        hbox.setSpacing(0)
        hbox.addWidget(self.bottomStackSwitcher)
        hbox.addStretch(1)
        hbox.addWidget(self.statusbar)
        mainLayout.addLayout(hbox)

        self.uptime = 0
        self.uptimeTimer = QtCore.QTimer()
        self.uptimeTimer.setInterval(60000)
        self.uptimeTimer.timeout.connect(self.updateUptime)
        self.uptimeTimer.start()

        # remember layout
        if projectPathDict['root'] in self.useData.OPENED_PROJECTS:
            settings = QtCore.QSettings("Clean Code Inc.", "Pcode")
            settings.beginGroup(projectPathDict['root'])
            if settings.value('hsplitter'):
                self.hSplitter.restoreState(settings.value('hsplitter'))
            if settings.value('vsplitter'):
                self.vSplitter.restoreState(settings.value('vsplitter'))
            if settings.value('sidesplitter'):
                self.sideSplitter.restoreState(settings.value('sidesplitter'))
            self.vSplitter.updateStatus()
            if settings.value('writepad'):
                self.writePad.setGeometry(settings.value('writepad'))
            settings.endGroup()

        self.setKeymap()

    def resizeView(self, hview, vview):
        hSizes = self.hSplitter.sizes()
        vSizes = self.vSplitter.sizes()
        if hview == 1:
            self.hSplitter.setSizes([hSizes[0] + 2, hSizes[1] - 2])
        elif hview == -1:
            self.hSplitter.setSizes([hSizes[0] - 2, hSizes[1] + 2])

        if vview == 1:
            self.vSplitter.setSizes([vSizes[0] + 2, vSizes[1] - 2])
        elif vview == -1:
            self.vSplitter.setSizes([vSizes[0] - 2, vSizes[1] + 2])

    def createActions(self):
        self.gotoLineAct = \
            QtWidgets.QAction(
                QtGui.QIcon(os.path.join("Resources", "images", "mail_check")),
                "Goto Line", self,
                statusTip="Goto Line", triggered=self.showGotoLineWidget)

        self.viewSwitcherAct = QtWidgets.QAction(
            "Switch Views", self, statusTip="Switch Views",
            triggered=self.showSnapShotSwitcher)

        self.exitAct = \
            QtWidgets.QAction("Exit", self, statusTip="Exit",
                          triggered=self.projects.closeProgram)

        # Menubar Actions ----------------------------------------------------

        #----------------------------------------------------------------------
        self.runFileAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "rerun")),
            "Run File", self,
            statusTip="Run current file", triggered=self.runFile)

        self.runProjectAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "run")),
            "Run Project", self,
            statusTip="Run Project", triggered=self.runProject)

        self.stopRunAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "stop")),
            "Stop", self,
            statusTip="Stop execution",
            triggered=self.stopProcess)

        self.runParamAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "shell")),
            "Set Run Parameters", self,
            statusTip="Set Run Parameters",
            triggered=self.setRunParameters)

        #---------------------------------------------------------------------

        self.finderAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "scope")),
            "Find", self,
            statusTip="Find", triggered=self.showFinderWidget)

        self.replaceAct = \
            QtWidgets.QAction(
                QtGui.QIcon(
                    os.path.join("Resources", "images", "edit-replace")),
                "Replace", self,
                statusTip="Replace",
                          triggered=self.showReplaceWidget)

        self.findInFilesAct = QtWidgets.QAction(
            QtGui.QIcon(os.path.join("Resources", "images", "find_in_files")),
            "Find-in-Files", self,
            statusTip="Find-in-Files", triggered=self.showFindInFilesWidget)

        self.clearRecentFilesAct = \
            QtWidgets.QAction(
                QtGui.QIcon(os.path.join("Resources", "images", "clear")),
                "Clear History", self, statusTip="Clear History",
                triggered=self.clearRecentFiles)

        self.writePadAct = \
            QtWidgets.QAction(
                QtGui.QIcon(os.path.join("Resources", "images", "pencil")),
                "Writepad", self, statusTip="Writepad",
                triggered=self.showWritePad)

        self.buildAct = \
            QtWidgets.QAction(
                "Build", self,
                statusTip="Build",
                triggered=self.buildProject)

        self.exportProjectAct = \
            QtWidgets.QAction(
                QtGui.QIcon(os.path.join("Resources", "images", "archive")),
                "Export as Zip...", self, statusTip="Export as Zip",
                triggered=self.exportProject)

        self.closeProjectAct = \
            QtWidgets.QAction(
                QtGui.QIcon(
                    os.path.join("Resources", "images", "inbox--minus")),
                "Close Project", self, statusTip="Close Project",
                triggered=self.closeProject)

    def showProjectConfiguration(self):
        self.editorTabWidget.showProjectConfiguration()

    def buildProject(self):
        self.projectManager.buildProject()

    def exportProject(self):
        self.projectManager.exportProject()

    def closeProject(self):
        self.projects.closeProject()

    def updateEncodingLabel(self, text):
        self.encodingLabel.setText(text)

    def showGotoLineWidget(self):
        self.editorTabWidget.showGotoLineWidget()

    def showSnapShotSwitcher(self):
        self.editorTabWidget.showSnapShotSwitcher()

    def addBottomWidget(self, widget, icon, name):
        self.bottomStack.addWidget(widget)
        self.bottomStackSwitcher.addButton(toolTip=name, icon=icon)

    def showWritePad(self):
        self.writePad.show()

    def showFinderWidget(self):
        self.findInFiles.dashboard.hide()
        self.searchWidget.showFinder()

    def showReplaceWidget(self):
        self.findInFiles.dashboard.hide()
        self.searchWidget.showReplaceWidget()

    def showFindInFilesWidget(self):
        self.searchWidget.hide()
        self.findInFiles.dashboard.show()

    def createToolbars(self):

        self.editorMenuButton = QtWidgets.QToolButton()
        self.editorMenuButton.setText("Menu")
        self.editorMenuButton.setToolButtonStyle(2)
        self.editorMenuButton.setAutoRaise(True)
        self.editorMenuButton.setPopupMode(2)
        self.editorMenuButton.setIcon(QtGui.QIcon(
            os.path.join("Resources", "images", "Dashboard")))
        self.editorMenuButton.setMenu(self.mainMenu)

        self.standardToolbar.addWidget(self.editorMenuButton)
        self.standardToolbar.addAction(self.editorTabWidget.openFileAct)
        self.standardToolbar.addAction(self.editorTabWidget.newPythonFileAct)
        self.standardToolbar.addSeparator()
        self.standardToolbar.addAction(self.editorTabWidget.saveAct)
        self.standardToolbar.addAction(self.editorTabWidget.saveAllAct)
        self.standardToolbar.addAction(self.editorTabWidget.undoAct)
        self.editorTabWidget.undoAct.setDisabled(True)
        self.standardToolbar.addAction(self.editorTabWidget.redoAct)
        self.editorTabWidget.redoAct.setDisabled(True)
        self.standardToolbar.addSeparator()
        self.standardToolbar.addAction(self.editorTabWidget.cutAct)
        self.editorTabWidget.cutAct.setDisabled(True)
        self.standardToolbar.addAction(self.editorTabWidget.copyAct)
        self.editorTabWidget.copyAct.setDisabled(True)
        self.standardToolbar.addAction(self.editorTabWidget.pasteAct)
        self.standardToolbar.addSeparator()
        self.standardToolbar.addAction(self.editorTabWidget.dedentAct)
        self.standardToolbar.addAction(self.editorTabWidget.indentAct)

        self.standardToolbar.addSeparator()
        self.standardToolbar.addAction(self.runFileAct)
        self.standardToolbar.addAction(self.runProjectAct)
        self.standardToolbar.addAction(self.stopRunAct)
        self.stopRunAct.setVisible(False)
        self.standardToolbar.addAction(self.runParamAct)
        self.standardToolbar.addSeparator()
        self.standardToolbar.addAction(self.finderAct)
        self.standardToolbar.addAction(self.replaceAct)
        self.standardToolbar.addAction(self.findInFilesAct)
        self.standardToolbar.addSeparator()
        self.standardToolbar.addAction(self.writePadAct)

        self.bookmarkToolbar.addAction(
            self.editorTabWidget.findNextBookmarkAct)
        self.bookmarkToolbar.addAction(
            self.editorTabWidget.findPrevBookmarkAct)
        self.bookmarkToolbar.addAction(self.editorTabWidget.removeBookmarksAct)

        self.standardToolbar.addAction(self.editorTabWidget.convertToAsAct)
        self.standardToolbar.addWidget(self.bookmarkToolbar)

    def recentFileActivated(self, action):
        path = action.text().split('  ', 1)[1]
        if os.path.exists(path):
            self.editorTabWidget.loadfile(path)
        else:
            message = QtWidgets.QMessageBox.warning(self, "Open",
                                                "File is unavailable!")

    def loadRecentFiles(self):
        if len(self.projectData['recentfiles']) > 0:
            self.recentFile_actionGroup = QtWidgets.QActionGroup(self)
            self.recentFile_actionGroup.triggered.connect(
                self.recentFileActivated)
            self.recentFilesMenu.clear()
            c = 1
            for i in self.projectData['recentfiles']:
                action = QtWidgets.QAction(str(c) + '  ' + i, self)
                self.recentFile_actionGroup.addAction(action)
                self.recentFilesMenu.addAction(action)
                c += 1
            self.recentFilesMenu.addSeparator()
            self.recentFilesMenu.addAction(self.clearRecentFilesAct)
        else:
            self.recentFilesMenu.addAction("No Recent Files")

    def updateRecentFiles(self, filePath):
        if filePath in self.projectData['recentfiles']:
            self.projectData['recentfiles'].remove(filePath)
            self.projectData['recentfiles'].insert(0, filePath)
        else:
            if len(self.projectData['recentfiles']) < 15:
                self.projectData['recentfiles'].insert(0, filePath)
            else:
                del self.projectData['recentfiles'][-1]
                self.projectData['recentfiles'].insert(0, filePath)
        self.loadRecentFiles()

    def clearRecentFiles(self):
        self.projectData['recentfiles'] = []
        self.recentFilesMenu.clear()
        self.loadRecentFiles()
        self.messagesWidget.addMessage(0, 'Recent Files:',
                                       ["Recent files history has been cleared!"])

    # def addToLibrary(self):
    #     self.library.addToLibrary(self.editorTabWidget)

    def updateUptime(self):
        self.uptime += 1
        if self.uptime == 60:
            new_time = "1hr"
        elif self.uptime > 60:
            t = int(str(self.uptime / 60).split('.')[0])
            h = str(t) + "hr"
            m = str(self.uptime - (t * 60)) + "min"
            new_time = h + m
        else:
            new_time = str(self.uptime) + "min"
        self.uptimeLabel.setText("Uptime: " + new_time)

    def saveAll(self):
        self.editorTabWidget.saveAll()

    def fileUrl(self, fname):
        """Select the right file url scheme according to the operating system"""
        if os.name == 'nt':
            # Local file
            if re.search(r'^[a-zA-Z]:', fname):
                return 'file:///' + fname
            # UNC based path
            else:
                return 'file://' + fname
        else:
            return 'file://' + fname

    def getPythonDocPath(self):
        """
        Return Python documentation path
        (Windows: return the PythonXX.chm path if available)
        """
        if os.name == 'nt':
            path = os.path.dirname(
                self.projectData['settings']["DefaultInterpreter"])
            doc_path = os.path.join(path, "Doc")
            if not os.path.isdir(doc_path):
                return
            python_chm = [path for path in os.listdir(doc_path)
                          if re.match(r"(?i)Python[0-9]{3}.chm", path)]
            if python_chm:
                return self.fileUrl(os.path.join(doc_path, python_chm[0]))
        else:
            vinf = sys.version_info
            doc_path = '/usr/share/doc/python%d.%d/html' % (vinf[0], vinf[1])
        python_doc = os.path.join(doc_path, "index.html")
        if os.path.isfile(python_doc):
            return self.fileUrl(python_doc)


    def setRunParameters(self):
        self.editorTabWidget.showSetRunParameters()

    def runFile(self):
        self.runWidget.runFile()

    def runProject(self):
        self.runWidget.runProject()

    def stopProcess(self):
        self.runWidget.stopProcess()

    def showPythonInterpreter(self):
        process = QtCore.QProcess()
        process.startDetached(self.useData.SETTINGS["DefaultInterpreter"])

    def showCommandPrompt(self):
        prompt = os.environ["COMSPEC"]
        process = QtCore.QProcess()
        process.startDetached(prompt, [], QtCore.QDir().rootPath())

    def showCursorPosition(self):
        line, index = self.editorTabWidget.currentEditor.getCursorPosition()
        self.cursorPositionButton.setText(
            "Line {0} : Column {1}".format(line + 1, index + 1))

    def updateLineCount(self, lines):
        self.linesLabel.setText("Lines: " + str(lines))

    def saveUiState(self):
        name = self.projectPathDict["root"]
        settings = QtCore.QSettings("Clean Code Inc.", "Pcode")
        settings.beginGroup(name)
        settings.setValue('hsplitter', self.hSplitter.saveState())
        settings.setValue('vsplitter', self.vSplitter.saveState())
        settings.setValue('sidesplitter', self.sideSplitter.saveState())
        settings.setValue('writepad', self.writePad.geometry())
        settings.endGroup()

    def restoreSession(self):
        self.editorTabWidget.restoreSession()

    def closeWindow(self):
        if self.runWidget.currentProcess is not None:
            mess = "Close running program?"
            reply = QtWidgets.QMessageBox.warning(self, "Close",
                                              mess, QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
            if reply == QtWidgets.QMessageBox.Yes:
                self.runWidget.stopProcess()
            else:
                return False
        modified = []
        for i in range(self.editorTabWidget.count()):
            if self.editorTabWidget.getEditor(i).isModified():
                modified.append(i)
        if len(modified) == 0:
            pass
        else:
            for i in range(len(modified)):
                v = modified.pop(-1)
                self.editorTabWidget.setCurrentIndex(v)
                mess = 'Save changes to "{0}"?'.format(
                    self.editorTabWidget.tabText(v))
                reply = QtWidgets.QMessageBox.warning(self, "Close", mess,
                                                  QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No |
                                                  QtWidgets.QMessageBox.Cancel)
                if reply == QtWidgets.QMessageBox.No:
                    if len(modified) == 0:
                        pass
                elif reply == QtWidgets.QMessageBox.Yes:
                    saved = self.editorTabWidget.save()
                    if saved:
                        pass
                    else:
                        return False
                elif reply == QtWidgets.QMessageBox.Cancel:
                    return False
        self.saveUiState()
        self.editorTabWidget.saveSession()
        self.projectData["settings"]["Closed"] = "True"
        self.saveProjectData()
        self.editorTabWidget.refactor.closeRope()

        return True

    def loadProjectData(self):
        dom_document = QtXml.QDomDocument()
        file = open(os.path.join(self.projectPathDict[
                    "root"], "Data", "projectdata.xml"), "r")
        x = dom_document.setContent(file.read())
        file.close()

        elements = dom_document.documentElement()
        node = elements.firstChild()

        shortcuts = []
        recentfiles = []
        favourites = []
        launchers = {}

        settingsList = []
        while node.isNull() is False:
            property = node.toElement()
            sub_node = property.firstChild()
            while sub_node.isNull() is False:
                sub_prop = sub_node.toElement()
                if node.nodeName() == "shortcuts":
                    shortcuts.append(sub_prop.text())
                elif node.nodeName() == "recentfiles":
                    if os.path.exists(sub_prop.text()):
                        recentfiles.append(sub_prop.text())
                    else:
                        pass
                elif node.nodeName() == "favourites":
                    favourites.append(sub_prop.text())
                elif node.nodeName() == "settings":
                    settingsList.append((tuple(sub_prop.text().split('=', 1))))
                elif node.nodeName() == "launchers":
                    tag = sub_prop.toElement()
                    path = tag.attribute("path")
                    param = tag.attribute("param")
                    launchers[path] = param
                sub_node = sub_node.nextSibling()
            node = node.nextSibling()
        settingsDict = dict(settingsList)

        settingsDict['LastCloseSuccessful'] = settingsDict['Closed']
        settingsDict['Closed'] = "False"

        self.projectData = {}
        self.projectData["shortcuts"] = shortcuts
        self.projectData["favourites"] = favourites
        self.projectData["recentfiles"] = recentfiles
        self.projectData["settings"] = settingsDict
        self.projectData["launchers"] = launchers

        # in order that a crash can be reported
        self.saveProjectData()

    def saveProjectData(self):
        domDocument = QtXml.QDomDocument("projectdata")

        projectdata = domDocument.createElement("projectdata")
        domDocument.appendChild(projectdata)

        root = domDocument.createElement("shortcuts")
        projectdata.appendChild(root)

        for i in self.projectData['shortcuts']:
            tag = domDocument.createElement("shortcut")
            root.appendChild(tag)

            t = domDocument.createTextNode(i)
            tag.appendChild(t)

        root = domDocument.createElement("recentfiles")
        projectdata.appendChild(root)

        for i in self.projectData['recentfiles']:
            tag = domDocument.createElement("recent")
            root.appendChild(tag)

            t = domDocument.createTextNode(i)
            tag.appendChild(t)

        root = domDocument.createElement("favourites")
        projectdata.appendChild(root)

        for i in self.projectData['favourites']:
            tag = domDocument.createElement("fav")
            root.appendChild(tag)

            t = domDocument.createTextNode(i)
            tag.appendChild(t)

        root = domDocument.createElement("launchers")
        projectdata.appendChild(root)

        for path, param in self.projectData['launchers'].items():
            tag = domDocument.createElement("item")
            tag.setAttribute("path", path)
            tag.setAttribute("param", param)
            root.appendChild(tag)

        root = domDocument.createElement("settings")
        projectdata.appendChild(root)

        s = 0
        for key, value in self.projectData['settings'].items():
            tag = domDocument.createElement("key")
            root.appendChild(tag)

            t = domDocument.createTextNode(key + '=' + value)
            tag.appendChild(t)
            s += 1

        path = os.path.join(
            self.projectPathDict["root"], "Data", "projectdata.xml")
        file = open(path, "w")
        file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        file.write(domDocument.toString())
        file.close()

    def setKeymap(self):
        shortcuts = self.useData.CUSTOM_SHORTCUTS

        self.shortGotoLine = QtWidgets.QShortcut(
            shortcuts["Ide"]["Go-to-Line"], self)
        self.shortGotoLine.activatedAmbiguously.connect(
            self.showGotoLineWidget)
        self.gotoLineAct.setShortcut(shortcuts["Ide"]["Go-to-Line"])

        self.shortBuild = QtWidgets.QShortcut(shortcuts["Ide"]["Build"], self)
        self.shortBuild.activatedAmbiguously.connect(self.buildProject)
        self.buildAct.setShortcut(shortcuts["Ide"]["Build"])

        self.shortFind = QtWidgets.QShortcut(shortcuts["Ide"]["Find"], self)
        self.shortFind.activated.connect(self.showFinderWidget)

        self.shortReplace = QtWidgets.QShortcut(
            shortcuts["Ide"]["Replace"], self)
        self.shortReplace.activated.connect(self.showReplaceWidget)

        self.shortRunFile = QtWidgets.QShortcut(
            shortcuts["Ide"]["Run-File"], self)
        self.shortRunFile.activated.connect(self.runFile)

        self.shortRunProject = QtWidgets.QShortcut(
            shortcuts["Ide"]["Run-Project"], self)
        self.shortRunProject.activated.connect(self.runProject)

        self.shortStopRun = QtWidgets.QShortcut(
            shortcuts["Ide"]["Stop-Execution"], self)
        self.shortStopRun.activated.connect(self.stopProcess)

