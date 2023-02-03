from PyQt5 import uic
from PyQt5.QtCore import QRegExp
from PyQt5.QtGui import QSyntaxHighlighter, QColor, QTextCharFormat, QFont, QKeySequence, QFontMetricsF
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QShortcut, QMessageBox
import serial
from threading import Thread, Lock
from time import sleep


def formatter(color, style=''):
    """Return a QTextCharFormat with the given attributes.
    """
    _color = QColor(255, 255, 255)
    _color.setNamedColor(color)

    _format = QTextCharFormat()
    _format.setForeground(_color)
    if 'bold' in style:
        _format.setFontWeight(QFont.Bold)
    if 'italic' in style:
        _format.setFontItalic(True)

    return _format


# Syntax styles that can be shared by all languages
STYLES = {
    'comment': formatter('green', 'italic'),
    'string': formatter('magenta'),
    'string2': formatter('magenta'),
    'keyword': formatter('blue'),
    'operator': formatter('red'),
    'brace': formatter('lightGray'),
    'defclass': formatter('white', 'bold'),
    'self': formatter('white', 'italic'),
    'numbers': formatter('yellow'),
}


class PythonHighlighter (QSyntaxHighlighter):
    """Syntax highlighter for the Python language.
    """
    # Python keywords
    keywords = [
        'and', 'assert', 'break', 'class', 'continue', 'def',
        'del', 'elif', 'else', 'except', 'exec', 'finally',
        'for', 'from', 'global', 'if', 'import', 'in',
        'is', 'lambda', 'not', 'or', 'pass', 'print',
        'raise', 'return', 'try', 'while', 'yield',
        'None', 'True', 'False',
    ]

    # Python operators
    operators = [
        '=',
        # Comparison
        '==', '!=', '<', '<=', '>', '>=',
        # Arithmetic
        '\+', '-', '\*', '/', '//', '\%', '\*\*',
        # In-place
        '\+=', '-=', '\*=', '/=', '\%=',
        # Bitwise
        '\^', '\|', '\&', '\~', '>>', '<<',
    ]

    # Python braces
    braces = [
        '\{', '\}', '\(', '\)', '\[', '\]',
    ]

    def __init__(self, document):
        QSyntaxHighlighter.__init__(self, document)

        # Multi-line strings (expression, flag, style)
        # FIXME: The triple-quotes in these two lines will mess up the
        # syntax highlighting from this point onward
        self.tri_single = (QRegExp("'''"), 1, STYLES['string2'])
        self.tri_double = (QRegExp('"""'), 2, STYLES['string2'])

        rules = []

        # Keyword, operator, and brace rules
        rules += [(r'\b%s\b' % w, 0, STYLES['keyword'])
            for w in PythonHighlighter.keywords]
        rules += [(r'%s' % o, 0, STYLES['operator'])
            for o in PythonHighlighter.operators]
        rules += [(r'%s' % b, 0, STYLES['brace'])
            for b in PythonHighlighter.braces]

        # All other rules
        rules += [
            # 'self'
            (r'\bself\b', 0, STYLES['self']),

            # 'def' followed by an identifier
            (r'\bdef\b\s*(\w+)', 1, STYLES['defclass']),
            # 'class' followed by an identifier
            (r'\bclass\b\s*(\w+)', 1, STYLES['defclass']),

            # From '#' until a newline
            (r'#[^\n]*', 0, STYLES['comment']),

            # Numeric literals
            (r'\b[+-]?[0-9]+[lL]?\b', 0, STYLES['numbers']),
            (r'\b[+-]?0[xX][0-9A-Fa-f]+[lL]?\b', 0, STYLES['numbers']),
            (r'\b[+-]?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?\b', 0, STYLES['numbers']),

            # Double-quoted string, possibly containing escape sequences
            (r'"[^"\\]*(\\.[^"\\]*)*"', 0, STYLES['string']),
            # Single-quoted string, possibly containing escape sequences
            (r"'[^'\\]*(\\.[^'\\]*)*'", 0, STYLES['string']),

        ]

        # Build a QRegExp for each pattern
        self.rules = [(QRegExp(pat), index, fmt)
                      for (pat, index, fmt) in rules]

    def highlightBlock(self, text):
        """Apply syntax highlighting to the given block of text.
        """
        # Do other syntax formatting
        for expression, nth, format in self.rules:
            index = expression.indexIn(text, 0)

            while index >= 0:
                # We actually want the index of the nth match
                index = expression.pos(nth)
                length = len(expression.cap(nth))
                self.setFormat(index, length, format)
                index = expression.indexIn(text, index + length)

        self.setCurrentBlockState(0)

        # Do multi-line strings
        in_multiline = self.match_multiline(text, *self.tri_single)
        if not in_multiline:
            in_multiline = self.match_multiline(text, *self.tri_double)

    def match_multiline(self, text, delimiter, in_state, style):
        """Do highlighting of multi-line strings. ``delimiter`` should be a
        ``QRegExp`` for triple-single-quotes or triple-double-quotes, and
        ``in_state`` should be a unique integer to represent the corresponding
        state changes when inside those strings. Returns True if we're still
        inside a multi-line string when this function is finished.
        """
        # If inside triple-single quotes, start at 0
        if self.previousBlockState() == in_state:
            start = 0
            add = 0
        # Otherwise, look for the delimiter on this line
        else:
            start = delimiter.indexIn(text)
            # Move past this match
            add = delimiter.matchedLength()

        # As long as there's a delimiter match on this line...
        while start >= 0:
            # Look for the ending delimiter
            end = delimiter.indexIn(text, start + add)
            # Ending delimiter on this line?
            if end >= add:
                length = end - start + add + delimiter.matchedLength()
                self.setCurrentBlockState(0)
            # No; multi-line string
            else:
                self.setCurrentBlockState(in_state)
                length = len(text) - start + add
            # Apply formatting
            self.setFormat(start, length, style)
            # Look for the next match
            start = delimiter.indexIn(text, start + length)

        # Return True if still inside a multi-line string, False otherwise
        if self.currentBlockState() == in_state:
            return True
        else:
            return False













class App(QApplication):
    def __init__(self):
        super().__init__([])
        Form, Window = uic.loadUiType("mainwindow.ui")
        self.window = Window()
        self.form = Form()
        self.form.setupUi(self.window)
        self.serial = None
        self.serial_dev = None
        self.serial_lock = Lock()
        self.old_values = None
        self.amperes_thread = Thread(target=self.read_amperes_thread)
        self.amperes_thread_running = False
        self.form.textEdit_code.highlight = PythonHighlighter(self.form.textEdit_code.document())
        self.form.textEdit_code.setTextColor(QColor(255, 255, 255))
        self.form.textEdit_code.setStyleSheet("QWidget{color: white; background-color: black;}");


        self.form.pushButton_connect.clicked.connect(self.connect)
        self.form.horizontalSlider_X.valueChanged.connect(self.update_slider_position)
        self.form.horizontalSlider_Y.valueChanged.connect(self.update_slider_position)
        self.form.horizontalSlider_Z.valueChanged.connect(self.update_slider_position)

        self.form.lineEdit_X.editingFinished.connect(self.update_lineedit_position)
        self.form.lineEdit_Y.editingFinished.connect(self.update_lineedit_position)
        self.form.lineEdit_Z.editingFinished.connect(self.update_lineedit_position)

        self.form.pushButton_send.clicked.connect(self.send)
        self.form.pushButton_runCode.clicked.connect(self.run_code)
        self.form.checkBox_autosend.clicked.connect(self.autosend)

        self.form.msgSc = QShortcut(QKeySequence('Ctrl+R'), self.form.pushButton_runCode)

        self.form.msgSc.activated.connect(self.run_code)

        self.form.textEdit_code.setTabStopDistance(
            QFontMetricsF(self.form.textEdit_code.font()).horizontalAdvance(' ') * 4)

        try:
            self.form.textEdit_code.setText(open("last_code.py", "r").read())
        except:
            pass
        self.form.textEdit_code.textChanged.connect(self.save_code)
        self.window.show()

    def connect(self):
        self.form.label_connectionStatus.setText("Connecting")
        success = False
        num = 0
        self.serial = serial.Serial(port=None, baudrate=57600)
        self.serial.dts = True
        while not success:
            try:
                self.serial.port = f'/dev/ttyACM{num}'
                self.serial.open()
                self.serial_dev = f'/dev/ttyACM{num}'
                success = True
                break
            except:
                print(f"Failed to connect to /dev/ttyACM{num}")
                if num > 100:
                    self.form.label_connectionStatus.setText("Disconnected")
                    break
            try:
                self.serial.port = f'/dev/ttyUSB{num}'
                self.serial.open()
                self.serial_dev = f'/dev/ttyUSB{num}'
                success = True
                break
            except:
                print(f"Failed to connect to /dev/ttyUSB{num}")
                num += 1
                if num > 100:
                    self.form.label_connectionStatus.setText("Disconnected")
                    break
        if not success:
            self.serial = None
        else:
            if not self.amperes_thread_running:
                self.amperes_thread_running = True
                self.amperes_thread.start()
            self.form.label_connectionStatus.setText(f"Connected on \n{self.serial_dev}")
            self.serial.timeout = 0.1

    def update_slider_position(self):
        self.form.lineEdit_X.setText(str(self.form.horizontalSlider_X.value()))
        self.form.lineEdit_Y.setText(str(self.form.horizontalSlider_Y.value()))
        self.form.lineEdit_Z.setText(str(self.form.horizontalSlider_Z.value()))

    def update_lineedit_position(self):
        try:
            self.form.horizontalSlider_X.setValue(int(self.form.lineEdit_X.text()))
        except ValueError:
            self.form.lineEdit_X.setText(str(self.form.horizontalSlider_X.value()))
        try:
            self.form.horizontalSlider_Y.setValue(int(self.form.lineEdit_Y.text()))
        except ValueError:
            self.form.lineEdit_Y.setText(str(self.form.horizontalSlider_Y.value()))
        try:
            self.form.horizontalSlider_Z.setValue(int(self.form.lineEdit_Z.text()))
        except ValueError:
            self.form.lineEdit_Z.setText(str(self.form.horizontalSlider_Z.value()))

    def autosend(self):
        Thread(target=self.send_thread_func).start()

    def send_thread_func(self):
        while self.form.checkBox_autosend.isChecked():
            sleep(0.1)
            self.send()

    def send(self):
        if self.serial is None:
            return
        x = int(self.form.lineEdit_X.text())
        y = int(self.form.lineEdit_Y.text())
        z = int(self.form.lineEdit_Z.text())
        pump = 1 if self.form.pushButton_pump.isChecked() else 0
        valve = 1 if self.form.pushButton_valve.isChecked() else 0
        values = (x, y, z, pump, valve)

        if self.old_values != values:
            self.send_values(values)

    def send_values(self, values):
        print("Acquiring serial lock")
        if self.serial is None:
            return
        if not self.serial_lock.acquire(True, 0.1):
            print("Timeout")
            return
        print("Sending")
        if self.serial.out_waiting == 0:
            b = bytes([255, ord('P')]) + bytes(" ".join([str(i) for i in values])+" ", "utf-8")
            self.serial.write(b)
            self.serial.flush()
            self.old_values = values
        self.serial_lock.release()
        print("Sent")

    def read_amperes(self):
        if not self.serial_lock.acquire(True, 0.1):
            return
        self.serial.write(bytes([255, ord('R')]))
        self.serial.flush()
        c = self.serial.read(1)
        s = bytes("", "ascii")
        while c not in (b'\n', b'\r'):
            s += c
            c = self.serial.read(1)
        while c in (b'\n', b'\r'):
            c = self.serial.read(1)
        s2 = bytes("", "ascii")
        while c not in (b'\n', b'\r'):
            s2 += c
            c = self.serial.read(1)
        self.serial.flushInput()
        s = s.decode("ascii")
        s2 = s2.decode("ascii")
        self.serial_lock.release()
        self.form.label_currentSensor.setText(str(s)+" "+str(s2))


    def read_amperes_thread(self):
        sleep(3)
        while self.amperes_thread_running:
            sleep(0.2)
            self.read_amperes()

    def run_code(self):
        code = self.form.textEdit_code.toPlainText()
        exec(code, globals(), locals())

    def save_code(self):
        code = self.form.textEdit_code.toPlainText()
        f = open("last_code.py", "w")
        f.write(code)
        f.close()



app = App()
app.exec_()


