"""
DESIGN TOKENS (DESIGN.md — ELIO Design System, light mode)
-----------------------------------------------------------
Colors, fonts and Qt stylesheets. Other modules that need the live value of
COMPACT (set once, from the screen size, before the main window is built)
must read it as `theme.COMPACT` — a `from .theme import COMPACT` copies the
default at import time and will not see the later update.
"""
import pyvista as pv

C = {
    "background": "#f8faf9",
    "surface_lowest": "#ffffff",      # cards
    "surface_low": "#f2f4f3",         # input troughs, camera sky
    "surface_container": "#eceeed",   # chips
    "surface_high": "#e6e9e8",        # dividers, tracks
    "surface_dim": "#d8dad9",         # card outlines, floor grid
    "on_surface": "#191c1c",
    "on_surface_variant": "#3d4943",
    "inverse_surface": "#2e3131",
    "inverse_on_surface": "#eff1f0",
    "outline": "#6d7a73",
    "outline_variant": "#bccac1",
    "primary": "#00694c",             # Eco Green
    "primary_container": "#008560",
    "on_primary": "#ffffff",
    "primary_fixed": "#86f8c9",
    "primary_fixed_dim": "#68dbae",
    "on_primary_fixed_variant": "#00513a",
    "tertiary": "#00647c",            # Vibrant Cyan: vision / data states
    "tertiary_container": "#007f9c",
    "error": "#ba1a1a",
    "error_container": "#ffdad6",
    # Not in DESIGN.md: caution state (near a limit, inverted axis)
    "warning": "#9a6700",
}
FONT_HEAD = "'Sora', 'Segoe UI Semibold', 'Segoe UI', sans-serif"
# Page title only. Loaded from fonts/ZenDots-Regular.ttf in app.py if not installed.
FONT_BRAND = "'Zen Dots', 'Sora', 'Segoe UI Semibold', sans-serif"
FONT_BODY = "'Inter', 'Segoe UI', sans-serif"
FONT_LABEL = "'Manrope', 'Segoe UI', sans-serif"
MONO = "'JetBrains Mono', 'Cascadia Mono', Consolas, monospace"

pv.global_theme.background = C["surface_lowest"]

STYLE_SHEET = f"""
QMainWindow, QDialog {{ background-color: {C['background']}; }}
QWidget {{
    color: {C['on_surface']};
    font-family: {FONT_BODY};
    font-size: 13px;
    font-weight: 500;
}}
QWidget#Page {{ background-color: {C['background']}; }}
QLabel {{ background: transparent; border: none; }}
QFrame#Card {{
    background-color: {C['surface_lowest']};
    border: 1px solid {C['surface_dim']};
    border-radius: 16px;
}}
QFrame#Pill {{
    background-color: {C['surface_lowest']};
    border: 1px solid {C['surface_dim']};
    border-radius: 16px;
}}
QFrame#Divider {{ background-color: {C['surface_high']}; border: none; }}
QLabel#PageTitle {{ font-family: {FONT_BRAND}; font-size: 22px; font-weight: 400; color: {C['primary']}; }}
QLabel#PageSubtitle {{ font-family: {FONT_LABEL}; font-size: 12px; font-weight: 600; color: {C['outline']}; }}
QLabel#CardTitle {{ font-family: {FONT_HEAD}; font-size: 15px; font-weight: 600; }}
QLabel#CardAction {{
    font-family: {FONT_LABEL}; font-size: 11px; font-weight: 600;
    color: {C['on_surface_variant']};
    background-color: {C['surface_container']};
    border-radius: 9px;
    padding: 2px 8px;
}}
QLabel#Muted {{ font-family: {FONT_LABEL}; font-size: 11px; font-weight: 600; color: {C['outline']}; }}
QLabel#Secondary {{ font-family: {FONT_LABEL}; font-size: 12px; font-weight: 600; color: {C['on_surface_variant']}; }}
QLabel#GroupLabel {{ font-family: {FONT_LABEL}; font-size: 12px; font-weight: 700; color: {C['on_surface_variant']}; }}
QLabel#RowName {{ font-family: {FONT_LABEL}; font-size: 13px; font-weight: 700; }}
QLabel#Metric {{ font-size: 20px; font-weight: 600; font-family: {MONO}; }}
QLabel#MetricSmall {{ font-size: 13px; font-weight: 600; font-family: {MONO}; }}
QLabel#Mono {{ font-size: 11px; font-family: {MONO}; color: {C['on_surface_variant']}; }}

QToolTip {{
    background-color: {C['inverse_surface']};
    color: {C['inverse_on_surface']};
    border: none;
    padding: 4px 8px;
}}

QTextEdit {{
    background-color: {C['surface_low']};
    border: 1px solid {C['surface_dim']};
    border-radius: 8px;
    padding: 8px;
    font-family: {MONO};
    font-size: 11px;
    color: {C['on_surface_variant']};
    selection-background-color: {C['primary_fixed']};
    selection-color: {C['on_surface']};
}}
QTextEdit:focus {{ border-color: {C['primary']}; }}

QPushButton {{
    background-color: rgba(255, 255, 255, 0.72);
    border: 1px solid rgba(0, 105, 76, 0.35);
    border-radius: 8px;
    color: {C['on_surface']};
    min-height: 30px;
    padding: 0 14px;
    font-family: {FONT_LABEL};
    font-size: 12px;
    font-weight: 700;
}}
QPushButton:hover {{ background-color: rgba(0, 105, 76, 0.06); border-color: {C['primary']}; }}
QPushButton:pressed {{ background-color: rgba(0, 105, 76, 0.12); }}
QPushButton:disabled {{ color: {C['outline_variant']}; background-color: {C['surface_low']}; border-color: {C['surface_high']}; }}
QPushButton#Primary {{
    color: {C['on_primary']};
    background-color: {C['primary']};
    border-color: {C['primary']};
}}
QPushButton#Primary:hover {{ background-color: {C['primary_container']}; border-color: {C['primary_container']}; }}
QPushButton#Primary:pressed {{ background-color: {C['on_primary_fixed_variant']}; }}
QPushButton#Primary:disabled {{ color: {C['outline_variant']}; background-color: {C['surface_low']}; border-color: {C['surface_high']}; }}
QPushButton#Toggle:checked {{
    color: {C['on_primary']};
    background-color: {C['primary']};
    border-color: {C['primary']};
}}
QPushButton#Danger {{ color: {C['error']}; border-color: rgba(186, 26, 26, 0.40); }}
QPushButton#Danger:hover {{ background-color: {C['error_container']}; border-color: {C['error']}; }}
QPushButton#Danger:disabled {{ color: {C['outline_variant']}; background-color: {C['surface_low']}; border-color: {C['surface_high']}; }}
QPushButton#Pad {{ min-width: 44px; }}

QComboBox {{
    background-color: {C['surface_low']};
    border: 1px solid {C['surface_dim']};
    border-radius: 8px;
    min-height: 30px;
    padding: 0 10px;
    color: {C['on_surface']};
}}
QComboBox:focus, QComboBox:on {{ border-color: {C['primary']}; }}
QComboBox:disabled {{ color: {C['outline']}; }}
QComboBox::drop-down {{ border: none; width: 18px; }}
QComboBox QAbstractItemView {{
    background-color: {C['surface_lowest']};
    border: 1px solid {C['surface_dim']};
    selection-background-color: {C['surface_container']};
    selection-color: {C['primary']};
    outline: none;
}}

QWidget#Switch {{ font-family: {FONT_LABEL}; font-size: 12px; font-weight: 700; }}

QCheckBox {{ font-family: {FONT_LABEL}; color: {C['outline']}; font-size: 11px; font-weight: 700; spacing: 4px; }}
QCheckBox:checked {{ color: {C['warning']}; }}
QCheckBox::indicator {{
    width: 11px; height: 11px;
    border: 1px solid {C['outline_variant']};
    border-radius: 4px;
    background: {C['surface_lowest']};
}}
QCheckBox::indicator:checked {{ background: {C['warning']}; border-color: {C['warning']}; }}
QCheckBox:disabled {{ color: {C['surface_dim']}; }}

QSlider::groove:horizontal {{ height: 4px; background: {C['surface_high']}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {C['primary']}; border-radius: 2px; }}
QSlider::handle:horizontal {{
    background: {C['surface_lowest']};
    border: 2px solid {C['primary']};
    width: 10px; height: 10px;
    margin: -5px 0;
    border-radius: 7px;
}}

QScrollBar:vertical {{ background: transparent; width: 8px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {C['outline_variant']}; border-radius: 4px; min-height: 24px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: transparent; }}
"""

# Appended after STYLE_SHEET on small screens (e.g. 1366x768): later rules win.
COMPACT_STYLE_SHEET = f"""
QWidget {{ font-size: 12px; }}
QLabel#PageTitle {{ font-size: 18px; }}
QLabel#PageSubtitle {{ font-size: 11px; }}
QLabel#CardTitle {{ font-size: 13px; }}
QLabel#CardAction {{ font-size: 10px; padding: 1px 7px; border-radius: 8px; }}
QLabel#Muted {{ font-size: 10px; }}
QLabel#Secondary {{ font-size: 11px; }}
QLabel#GroupLabel {{ font-size: 11px; }}
QLabel#RowName {{ font-size: 12px; }}
QLabel#Metric {{ font-size: 16px; }}
QLabel#MetricSmall {{ font-size: 12px; }}
QLabel#Mono {{ font-size: 10px; }}
QWidget#Switch {{ font-size: 11px; }}
QFrame#Pill {{ border-radius: 13px; }}
QTextEdit {{ padding: 6px; font-size: 10px; }}
QPushButton {{ min-height: 24px; padding: 0 10px; font-size: 11px; }}
QPushButton#Pad {{ min-width: 34px; }}
QComboBox {{ min-height: 24px; padding: 0 8px; }}
"""

# Set from the screen size in app.py; small screens get the compact layout.
# Read/write this as `theme.COMPACT` from other modules (see module docstring).
COMPACT = False
COMPACT_MAX_W, COMPACT_MAX_H = 1440, 800
