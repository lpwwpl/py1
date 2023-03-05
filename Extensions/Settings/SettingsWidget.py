from PyQt5 import QtCore, QtGui, QtWidgets

from Extensions.Settings.ColorScheme.ColorScheme import ColorScheme
from Extensions.Settings.Keymap import Keymap
from Extensions.Settings.SnippetsManager import SnippetsManager
from Extensions.Settings.GeneralSettings import GeneralSettings
from Extensions.Settings.ModuleCompletion import ModuleCompletion


class SettingsWidget(QtWidgets.QDialog):

    def __init__(self, useData, mainApp, projectWindowStack, parent=None):
        QtWidgets.QDialog.__init__(self, parent, QtCore.Qt.Window |
                               QtCore.Qt.WindowCloseButtonHint)

        self.setWindowTitle("Settings")

        self.useData = useData
        self.projectWindowStack = projectWindowStack

        mainLayout = QtWidgets.QVBoxLayout()
        mainLayout.setContentsMargins(0,0,0,0)
        self.setLayout(mainLayout)

        self.settingsTab = QtWidgets.QTabWidget()
        self.settingsTab.setObjectName("settingsTab")
        mainLayout.addWidget(self.settingsTab)

        self.generalSettings = GeneralSettings(useData, mainApp, projectWindowStack)
        self.settingsTab.addTab(self.generalSettings, "General")

        self.snippetEditor = SnippetsManager(
            self.useData.appPathDict["snippetsdir"], self)
        self.settingsTab.addTab(self.snippetEditor, "Snippets")

        self.keymapWidget = Keymap(self.useData, projectWindowStack, self)
        self.settingsTab.addTab(self.keymapWidget, "Shortcuts")

        self.colorScheme = ColorScheme(self.useData, projectWindowStack)
        self.settingsTab.addTab(self.colorScheme, "Color Scheme")

        self.libraries = ModuleCompletion(self.useData)
        self.settingsTab.addTab(self.libraries, "Module Completion")
