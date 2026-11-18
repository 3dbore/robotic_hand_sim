"""Application entry point: builds the QApplication and shows the main window."""
import os
import sys

from PyQt5.QtGui import QFontDatabase, QIcon
from PyQt5.QtWidgets import QApplication

from . import config, theme
from .sim import ase


def setup():
    """Load saved settings and the effective joint limits (call before MainWindow)."""
    config.load_settings()
    config.apply_sim_limits(ase.JOINT_LIMITS)


def main():
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("ELIO.HILToolkit")

    app = QApplication(sys.argv)
    if os.path.exists(config.APP_ICON_FILE):
        app.setWindowIcon(QIcon(config.APP_ICON_FILE))
    if "Zen Dots" not in QFontDatabase().families() and os.path.exists(config.BRAND_FONT_FILE):
        QFontDatabase.addApplicationFont(config.BRAND_FONT_FILE)

    avail = app.primaryScreen().availableGeometry()
    theme.COMPACT = avail.width() <= theme.COMPACT_MAX_W or avail.height() <= theme.COMPACT_MAX_H

    setup()
    from .main_window import MainWindow
    window = MainWindow()
    window.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
