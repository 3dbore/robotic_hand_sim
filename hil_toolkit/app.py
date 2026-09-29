"""Application entry point: builds the QApplication and shows the main window."""
import os
import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QFontDatabase, QIcon

from . import theme
from .paths import BRAND_FONT_FILE, APP_ICON_FILE
from .main_window import HILToolkitGUI


def main():
    import ctypes
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("ELIO.HILToolkit")

    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon(APP_ICON_FILE))
    if "Zen Dots" not in QFontDatabase().families() and os.path.exists(BRAND_FONT_FILE):
        QFontDatabase.addApplicationFont(BRAND_FONT_FILE)

    avail = app.primaryScreen().availableGeometry()
    theme.COMPACT = avail.width() <= theme.COMPACT_MAX_W or avail.height() <= theme.COMPACT_MAX_H

    window = HILToolkitGUI()
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
