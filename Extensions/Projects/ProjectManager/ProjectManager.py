import os
import shutil
from PyQt5 import QtCore, QtGui, QtWidgets

from Extensions.Projects.ProjectManager.ProjectView.ProjectView import ProjectView


class ExportThread(QtCore.QThread):

    def run(self):
        self.error = None
        try:
            shutil.make_archive(self.fileName, "zip", self.path)
        except Exception as err:
            self.error = str(err)

    def export(self, fileName, path):
        self.fileName = fileName
        self.path = path

        self.start()


class ProjectManager(QtWidgets.QWidget):

    def __init__(
        self, editorTabWidget, messagesWidget, projectPathDict, projectSettings,
            useData, app,
            busyWidget, buildStatusWidget, parent):
        QtWidgets.QWidget.__init__(self, parent)

        self.busyWidget = busyWidget
        self.editorTabWidget = editorTabWidget

        self.useData = useData
        self.projects = parent
        self.projects = parent

        self.exportThread = ExportThread()
        self.exportThread.finished.connect(self.finishExport)

        self.projectView = ProjectView(
            self.editorTabWidget, projectPathDict["sourcedir"], app, projectSettings)

    def exportProject(self):
        curren_window = self.projects.projectWindowStack.currentWidget()
        name = curren_window.projectPathDict["name"]
        path = curren_window.projectPathDict["root"]

        options = QtWidgets.QFileDialog.Options()
        savepath = os.path.join(self.useData.getLastOpenedDir(), name)
        savepath = os.path.normpath(savepath)
        fileName = QtWidgets.QFileDialog.getSaveFileName(self,
                                                     "Export", savepath,
                                                     "All files (*)", options)
        if fileName:
            self.useData.saveLastOpenedDir(os.path.split(fileName)[0])

            self.exportThread.export(fileName, path)
            self.busyWidget.showBusy(True, "Exporting project... please wait!")

    def finishExport(self):
        self.busyWidget.showBusy(False)
        if self.exportThread.error is not None:
            message = QtWidgets.QMessageBox.warning(
                self, "Export Failed", self.exportThread.error)
