import keyword
import sys

from PyQt5 import QtGui, QtWidgets
from PyQt5.Qsci import QsciLexerCustom
from PyQt5.Qsci import QsciScintilla
import re
# Platform specific fonts
if sys.platform == 'win32':
    defaultFont = 'Consolas'
elif sys.platform == 'darwin':
    defaultFont = 'Monaco'
else:
    defaultFont = 'Bitstream Vera Sans Mono'


def defaultStyle():
    defaultStyle = {
        'Value': [defaultFont, QtGui.QColor("#099"), 10, False, False, '#ffffff'],
        'Param': [defaultFont, '#00cc00', 10, False, False, '#ffffff'],
        'Default': [defaultFont, '#000000', 10, False, False, '#ffffff'],
        'Function': [defaultFont, '#0000ff', 10, False, False, '#ffffff'],
        'DoubleQuotedString': [defaultFont, '#00aa00', 10, False, False, '#ffffff'],
        'Operator': [defaultFont, QtGui.QColor("#a11"), 10, False, False, '#ffffff'],
        'Number': [defaultFont, QtGui.QColor("#808"), 10, False, False, '#ffffff'],
        'Keyword': [defaultFont, '#0000ff', 10, False, False, '#ffffff'],
        'Variable': [defaultFont, QtGui.QColor("#808"), 10, False, False, '#ffffff'],
        'BuiltInFunction': [defaultFont, '#0000ff', 10, False, False, '#ffffff'],
        'SingleQuotedString': [defaultFont, '#00aa00', 10, False, False, '#ffffff'],
        'Comment': [defaultFont, QtGui.QColor("gray"), 10, False, False, '#ffffff'],
        'Folder': [defaultFont, '#0000ff', 10, False, False, '#ffffff']}

    return defaultStyle

class AsLexer(QsciLexerCustom):
    styles = {
        "Default": 0,
        "Comment": 1,
        "Keyword": 2,
        "BuiltInFunction": 3,
        "Variable": 4,
        "Number": 5,
        "Operator": 6,
        "Function": 7,
        "DoubleQuotedString": 8,
        "SingleQuotedString": 9,
        "Param": 10,
        "Value": 11,
        "Folder": 12,
    }
    builtin_function_list = [
        "BITS", "SIGNAL", "SWAIT", "PRINT", "TYPE", "BREAK","TWAIT",
        "TOOL", "BASE", "SPEED", "JMOVE", "LMOVE", "LAPPRO",
        "JAPPRO", "SIG", "ACCURACY", "DRAW", "TDRAW", "DRIVE",
        "HERE", "PAUSE", "POINT","POINT/OAT","POINT/X","POINT/Z","RETURN","LDEPART","HOME"
    ]
    keyword_list = [
         "IF", "ELSE", "UNTIL", "THEN",
        "WHILE", "FOR", "MOD", "BY", "TO", "<>","CASE","VALUE",
        "GOTO",
        "OR", "AND", "CALL","ALWAYS","FINE","MM/S","MM/MIN"
    ]
    folder_list = [
        "PROGRAM",  "TRANS", "JOINTS", "STRINGS", "REALS"
    ]
    # splitter = re.compile(r"\r|\n|\s+|\"|;|\'|\W|\w+|[\u0080-\uffff]+")
    splitter = re.compile(r"\r|\n|\s+|\"|;|\'|\w+|\W|[\u0080-\uffff]+")
    operatorList = ('[', ']', '(', ')',  ',', '#', '$', '=', "+", "-", "*", ".", "<", ">", '"', "'", ' ',"\r","\n")

    def __init__(self, style, paper):
        QsciLexerCustom.__init__(self)

        self.lexerPaper = paper

        for key, attrib in style.items():
            value = self.styles[key]
            self.setColor(QtGui.QColor(attrib[1]), value)
            self.setEolFill(True, value)
            self.setPaper(QtGui.QColor(attrib[5]), value)
            if self.lexerPaper[0] == "Plain":
                self.setPaper(QtGui.QColor(attrib[5]), value)
            else:
                self.setPaper(QtGui.QColor(self.lexerPaper[1]), value)

            font = QtGui.QFont(attrib[0], attrib[2])
            font.setBold(attrib[3])
            font.setItalic(attrib[4])
            self.setFont(font, value)

        if self.lexerPaper[0] == "Plain":
            self.setDefaultPaper(QtGui.QColor("#ffffff"))
        else:
            self.setDefaultPaper(QtGui.QColor(self.lexerPaper[1]))
        # self.setDefaultStyle()
        self._foldcompact = True

    def description(self, style):
        if style <= 11:
            description = "Custom lexer for the KAs programming languages"
        else:
            description = ""
        return description

    def language(self) -> str:
        return "K_As"

    def setKAsAutocomplete(selfself):
        pass

    def foldCompact(self):
        return self._foldcompact

    def setFoldCompact(self, enable):
        self._foldcompact = bool(enable)

    def isOperator(self, aChar):
        # if (ord(ch[0]) > 0x80 or not ch.isalnum()):
        if aChar == '':
            return False
        if not aChar[0].isascii():
            return False
        if aChar in self.operatorList:
            return True
        return False

    def isFloat(self,aChar):
        value = re.compile(r'(-?[0-9]+|(-?[0-9]*\.[0-9]+)([eE][-+]?[0-9]+)?)')
        result = value.match(aChar)
        return result

    def isAlphaNum(self,c):
        return (c >= '0' and c <= '9') or (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z')

    def iswordchar(self,c):
        return self.isAlphaNum(c) or c == '.' or c == '_'

    def isspacechar(self,c):
        asc = ord(c)
        return c == ' ' or (asc >= 0x09 and asc <= 0x0d)

    def styleText(self, start, end):
        editor = self.editor()
        if editor is None:
            return
        # Initialize the styling
        self.startStyling(start)

        text = bytearray(editor.text(), "utf-8")[start:end].decode("utf-8")
        setStyling = self.setStyling
        DEFAULT = self.styles["Default"]
        COMMENT = self.styles["Comment"]
        KEYWORD = self.styles["Keyword"]
        FOLDER_KEY = self.styles['Folder']
        BUILTINFUNCTION = self.styles["BuiltInFunction"]
        FUNCTION = self.styles["Function"]
        NUMBER = self.styles["Number"]
        OPERATOR = self.styles["Operator"]
        DOUBLEQUOTEDSTRING = self.styles["DoubleQuotedString"]
        SINGLEQUOTEDSTRING = self.styles["SingleQuotedString"]
        VARIABLE = self.styles["Variable"]
        VALUE = self.styles["Value"]
        PARAM = self.styles["Param"]

        tokens = [
            (token, len(bytearray(token, "utf-8")))
            for token in self.splitter.findall(text)
        ]
        state = -1
        lastState = -1  # before operator
        inBracket = False
        inParentheses = False

        for tokeni in tokens:
            token, count = tokeni

            # if state == COMMENT:
            #     if token == "\n":
            #         state = DEFAULT
            #     self.setStyling(count, state)
            if state == DOUBLEQUOTEDSTRING:
                if token == '"':
                    lastState, state = state, lastState
                self.setStyling(count, DOUBLEQUOTEDSTRING)
            elif state == SINGLEQUOTEDSTRING:
                if token == "'":
                    lastState, state = state, lastState
                self.setStyling(count, SINGLEQUOTEDSTRING)
            elif inParentheses:
                if token == ")":
                    inParentheses = False
                    lastState, state = state, lastState
                    self.setStyling(count, OPERATOR)
                else:
                    self.setStyling(count, PARAM)

            elif self.isOperator(token):
                opStyle = True  # 是否显示为Operator

                lastState = state
                if token == ";":  # 注释
                    state = COMMENT
                elif token == '"':
                    state = DOUBLEQUOTEDSTRING
                    opStyle = False
                elif token == "'":
                    state = SINGLEQUOTEDSTRING
                    opStyle = False
                elif token == "[":
                    state = PARAM
                    inBracket = True
                elif token == "]":
                    state = DEFAULT
                    inBracket = False
                elif token == "$":  # 变量
                    state = VARIABLE
                elif token == "#":
                    state = VARIABLE
                elif token == "(":
                    state = PARAM
                    inParentheses = True
                elif token == ")":
                    state = lastState
                    inParentheses = False
                elif token == "=":
                    state = VALUE
                elif token == ">" or token == "<":
                    lastState, state = state, lastState
                elif token == ' ':
                    state = DEFAULT
                elif token == '.':
                    if lastState == DEFAULT:
                        lastState,state = state,FOLDER_KEY
                    else:
                        pass
                elif token == ",":
                    if inBracket:
                        lastState, state = state, PARAM
                    else:
                        lastState, state = state, lastState
                elif "\r" in token or "\n" in token:
                    state = DEFAULT
                else:
                    state = lastState
                if opStyle:
                    self.setStyling(count, OPERATOR)
                else:
                    self.setStyling(count, state)
            elif state == FOLDER_KEY and token == 'END':
                self.setStyling(count, FOLDER_KEY)
            elif token in self.keyword_list:
                self.setStyling(count, KEYWORD)
            elif token in self.folder_list:
                self.setStyling(count, FOLDER_KEY)
            elif token in self.builtin_function_list:
                self.setStyling(count,BUILTINFUNCTION)
            elif self.isFloat(token):
                self.setStyling(count,NUMBER)
            elif token == ";":  # 注释
                self.setStyling(count,COMMENT)
                state = COMMENT
            elif token == '\n':
                state = DEFAULT
                self.setStyling(count, state)
            else:
                self.setStyling(count, state)

        SCI = editor.SendScintilla
        GETFOLDLEVEL = QsciScintilla.SCI_GETFOLDLEVEL
        SETFOLDLEVEL = QsciScintilla.SCI_SETFOLDLEVEL
        HEADERFLAG = QsciScintilla.SC_FOLDLEVELHEADERFLAG
        LEVELBASE = QsciScintilla.SC_FOLDLEVELBASE
        NUMBERMASK = QsciScintilla.SC_FOLDLEVELNUMBERMASK
        WHITEFLAG = QsciScintilla.SC_FOLDLEVELWHITEFLAG
        set_style = self.setStyling

        # source = ''
        # if end > editor.length():
        #     end = editor.length()
        # if end > start:
        #     source = bytearray(end - start + 1)
        #     SCI(QsciScintilla.SCI_GETTEXTRANGE, start, end, source)
        # if not source:
        #     return
        # 
        # length = end-start
        # lineCurrent = SCI(QsciScintilla.SCI_LINEFROMPOSITION, start)
        # if not lineCurrent:
        #     level = LEVELBASE
        #     lastLevel = level
        # else:
        #     lastLevel = SCI(GETFOLDLEVEL,lineCurrent-1)
        #     level = lastLevel
        # foldCompact = self.foldCompact()
        # visibleChars = 0
        # index = 0
        # while index < length:
        #     ch = chr(source[index])
        #     style = SCI(QsciScintilla.SCI_GETSTYLEAT, start+index)
        #     atEOL = (ch == '\n')
        #     atComment = ch == ';'
        #     if style == self.styles['Folder']:
        #         ba = bytearray()
        #         for j in range(10):
        #             if j+index >= length:
        #                 break
        #             if self.iswordchar(chr(source[j+index])) == False:
        #                 break
        #             ba.append(source[j+index])
        # 
        #         str = ba.decode()
        #         index += len(str)
        #         if len(str) > 0:
        #             print(str)
        #         if str in ['PROGRAM','STRINGS','JOINTS','REALS','TRANS']:
        #             level += 1
        #         elif str in ['END'] :
        #             level -= 1
        #     else:
        #         index+=1
        # 
        #     if atEOL:
        #         lev = lastLevel
        #         if visibleChars == 0 and foldCompact:
        #             lev |= WHITEFLAG
        #         elif level > lastLevel and visibleChars > 0:
        #             lev &= NUMBERMASK
        #             # lev |=HEADERFLAG
        #         # else:
        #         #     lev &= NUMBERMASK
        #         # if lev != SCI(GETFOLDLEVEL,lineCurrent):
        #         SCI(SETFOLDLEVEL, lineCurrent, lev)
        #         self.editor().foldLine(lineCurrent)
        # 
        #         lineCurrent += 1
        #         lastLevel = level
        #         visibleChars = 0
        #     if self.isspacechar(ch) == False:
        #         visibleChars += 1



    def keywords(self, set):
        # 1 to 9 sets
        if set == 1:
            k = keyword.kwlist
            s = k[0]
            for i in k[1:]:
                s += ' ' + i
            return s
        elif set == 2:
            s = ''
            for i in dir(__builtins__):
                s += ' ' + i
            return s
