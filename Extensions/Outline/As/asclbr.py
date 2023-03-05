
import re
import io
import tokenize
from token import NAME, DEDENT, OP
from PyQt5.QtCore import QRegExp
from PyQt5.QtCore import Qt

class Function:

    '''Class to represent a top-level Python function'''

    def __init__(self, name, lineno):
        self.name = name
        self.lineno = lineno

        self.objectType = "Function"

class GlobalVariable:

    '''Class to represent a top-level Python global variable'''

    def __init__(self, name, lineno):
        self.name = name
        self.lineno = lineno

        self.objectType = "GlobalVariable"

def _readmodule(source):
    outlineDict = {}

    f = io.StringIO(source)

    regex = QRegExp(".PROGRAM \\s*([^\\(]*)\\(([^)]*)\\)(.*)", Qt.CaseSensitive, QRegExp.RegExp)#,Qt.CaseSensitivity,QRegExp.RegExp
    if not regex.isValid():
        print("not regex.isValid()")
        return outlineDict
    try:
        lineno = 0
        bVarStat = False
        while True:
            sLine = f.readline()

            lineno = lineno+1
            if not sLine:

                break

            bFuncStat = regex.exactMatch(sLine)

            if sLine.startswith(".TRANS") or sLine.startswith(".JOINTS") or sLine.startswith(".STRINGS") or sLine.startswith(".REALS"):
                bVarStat = True
            if sLine.startswith(".END"):
                bVarStat = False
            if not bFuncStat and not bVarStat:
                continue
            if bFuncStat:
                nCount = regex.captureCount()
                if nCount < 3:
                    continue
                sMatch = regex.cap(0)
                sFunName = regex.cap(1)
                sFunArgs = regex.cap(2)

                outlineDict[sFunName]=1
                outlineDict[sFunName] = Function(sFunName, lineno)
            if bVarStat:
                strList = sLine.split(" ")
                if len(strList) > 1 and not strList[0].startswith(";"):
                    firstword = strList[0]

                    outlineDict[firstword] = GlobalVariable(firstword, lineno)
                    print(firstword)
    except Exception as e:
        print(e)
        pass

    f.close()
    return outlineDict
