import sys
import os
import zipfile
from PyQt5 import QtCore, QtGui,QtWidgets
# from PyQt5.QtGui import *
# from PyQt5.QtWidgets import *

class CreateWorkSpaceThread(QtCore.QThread):

    def run(self):
        self.errors = None
        try:
            zip = zipfile.ZipFile(
                os.path.join("Resources", "PcodeProjects.zip"), 'r')
            zip.extractall(self.path)
        except Exception as err:
            self.errors = str(err)

    def createWorkspace(self, path):
        self.path = path

        self.start()


class GetPathLine(QtWidgets.QWidget):

    textChanged = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)

        mainLayout = QtWidgets.QHBoxLayout()
        mainLayout.setContentsMargins(0,0,0,0)
        self.setLayout(mainLayout)

        self.destinationLine = QtWidgets.QLineEdit()
        self.destinationLine.textChanged.connect(self.textChanged.emit)
        mainLayout.addWidget(self.destinationLine)

        homePath = QtCore.QDir().homePath()

        # Todo: Workspace must unpack to the platform specific home
        # directory by default
        if sys.platform == 'win32':
            path = os.path.join(homePath,
                                "My Documents", "PcodeProjects")
        elif sys.platform == 'darwin':
            path = os.path.join(homePath,
                                "Documents", "PcodeProjects")
        else:
            path = os.path.join(homePath,
                                "My Documents", "PcodeProjects")
        path = os.path.normpath(path)
        self.destinationLine.setText(path)

        self.browseButton = QtWidgets.QPushButton('...')
        self.browseButton.clicked.connect(self.browsePath)
        mainLayout.addWidget(self.browseButton)

    def browsePath(self):
        homePath = QtCore.QDir().homePath()
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Folder",
            homePath)
        if directory:
            self.destinationLine.setText(os.path.normpath(directory))

    def text(self):
        return self.destinationLine.text()


class Workspace(QtWidgets.QDialog):

    def __init__(self, parent=None):
        QtWidgets.QDialog.__init__(self, parent,
                               QtCore.Qt.Window | QtCore.Qt.WindowCloseButtonHint)

        self.setWindowTitle("Workspace")
        self.setWindowIcon(
            QtGui.QIcon(os.path.join("Resources", "images", "Icon")))
        self.setFixedSize(500, 130)

        self.createWorkSpaceThread = CreateWorkSpaceThread()
        self.createWorkSpaceThread.finished.connect(self.completeWorkspace)

        mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(mainLayout)

        mainLayout.addWidget(
            QtWidgets.QLabel("Choose the location of your Workspace:"))

        self.choiceBox = QtWidgets.QComboBox()
        self.choiceBox.addItem("Choose an existing one")
        self.choiceBox.addItem("Create new")
        mainLayout.addWidget(self.choiceBox)

        self.getPathLine = GetPathLine()
        mainLayout.addWidget(self.getPathLine)

        mainLayout.addStretch(1)

        hbox = QtWidgets.QHBoxLayout()
        mainLayout.addLayout(hbox)

        self.statusLabel = QtWidgets.QLabel()
        hbox.addWidget(self.statusLabel)

        hbox.addStretch(1)

        self.okButton = QtWidgets.QPushButton("Done")
        self.okButton.clicked.connect(self.accept)
        hbox.addWidget(self.okButton)

        self.cancelButton = QtWidgets.QPushButton("Cancel")
        self.cancelButton.clicked.connect(self.cancel)
        hbox.addWidget(self.cancelButton)

        self.created = False

        self.exec_()

    def completeWorkspace(self):
        QtWidgets.QApplication.restoreOverrideCursor()
        if self.createWorkSpaceThread.errors is None:
            self.path = os.path.join(
                self.createWorkSpaceThread.path, "PcodeProjects")
            self.created = True
            self.close()
        else:
            self.statusLabel.clear()
            message = QtWidgets.QMessageBox.warning(
                self, "Workspace", "Error creating workspace:\n\n{0}".format(self.createWorkSpaceThread.errors))
            self.okButton.setDisabled(False)
            self.cancelButton.setDisabled(False)
            self.getPathLine.setDisabled(False)
            self.choiceBox.setDisabled(False)

    def accept(self):
        path = self.getPathLine.text()
        if os.path.exists(path):
            if self.choiceBox.currentIndex() == 0:
                if os.path.basename(path) == "PcodeProjects":
                    self.path = path
                    self.created = True
                    self.close()
                else:
                    message = QtWidgets.QMessageBox.warning(
                        self, "Workspace", "The workspace is not valid!")
                    return
            else:
                self.okButton.setDisabled(True)
                self.cancelButton.setDisabled(True)
                self.getPathLine.setDisabled(True)
                self.choiceBox.setDisabled(True)
                self.statusLabel.setText("Creating workspace...")
                QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)

                self.createWorkSpaceThread.createWorkspace(path)
        else:
            message = QtWidgets.QMessageBox.warning(
                self, "Workspace", "Path does not exist.")

    def cancel(self):
        self.created = False
        self.close()
