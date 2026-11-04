import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

# Palette
BG = "#14161c"
PANEL = "#1d2029"
KEY = "#2a2e3b"
KEY_ACTIVE = "#4f8cff"
TEXT = "#e6e8ee"
MUTED = "#8b91a3"
OK = "#3ddc84"
BAD = "#ff5c5c"
WARN = "#ffb84d"

# key -> (serial command, label, caption)
KEYS = {
    'w': ('F', "W", "Forward"),
    'a': ('L', "A", "Left"),
    's': ('B', "S", "Back"),
    'd': ('R', "D", "Right"),
}


class CarController:
    def __init__(self, root):
        self.root = root
        self.root.title("4WD Car Controller")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.serial_conn = None
        self.held = []  # held WASD keys, most recent last
        self.key_widgets = {}

        self._build_style()
        self._build_ui()
        self.scan_ports()
        self._bind_keyboard()

    def _build_style(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED, font=("Segoe UI", 9))
        style.configure("TButton", background=KEY, foreground=TEXT, borderwidth=0,
                        focusthickness=0, padding=(12, 6), font=("Segoe UI", 10))
        style.map("TButton", background=[("active", KEY_ACTIVE), ("disabled", PANEL)],
                  foreground=[("disabled", MUTED)])
        style.configure("TCombobox", fieldbackground=KEY, background=KEY, foreground=TEXT,
                        arrowcolor=TEXT, borderwidth=0)
        style.map("TCombobox", fieldbackground=[("readonly", KEY)], foreground=[("readonly", TEXT)])
        self.root.option_add("*TCombobox*Listbox.background", KEY)
        self.root.option_add("*TCombobox*Listbox.foreground", TEXT)

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill="both", expand=True)

        # Connection card
        card = ttk.Frame(outer, style="Card.TFrame", padding=14)
        card.pack(fill="x")

        ttk.Label(card, text="Serial Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(card, state="readonly", width=30)
        self.port_combo.grid(row=1, column=0, sticky="we", pady=(4, 0))
        card.columnconfigure(0, weight=1)

        ttk.Button(card, text="Scan", command=self.scan_ports).grid(row=1, column=1, padx=(8, 0), pady=(4, 0))
        self.connect_btn = ttk.Button(card, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=(8, 0), pady=(4, 0))

        status = ttk.Frame(card, style="Card.TFrame")
        status.grid(row=2, column=0, columnspan=3, sticky="w", pady=(10, 0))
        self.status_dot = tk.Canvas(status, width=12, height=12, bg=PANEL, highlightthickness=0)
        self.status_dot.pack(side="left")
        self.dot = self.status_dot.create_oval(2, 2, 10, 10, fill=BAD, outline="")
        self.status_label = ttk.Label(status, text="Disconnected", style="Muted.TLabel")
        self.status_label.pack(side="left", padx=(6, 0))

        # WASD pad
        pad = ttk.Frame(outer, style="Card.TFrame", padding=22)
        pad.pack(fill="x", pady=(14, 0))
        grid = ttk.Frame(pad, style="Card.TFrame")
        grid.pack()

        positions = {'w': (0, 1), 'a': (1, 0), 's': (1, 1), 'd': (1, 2)}
        for key, (row, col) in positions.items():
            cmd, label, caption = KEYS[key]
            cell = tk.Frame(grid, bg=KEY, width=92, height=92, cursor="hand2")
            cell.grid(row=row, column=col, padx=5, pady=5)
            cell.pack_propagate(False)
            big = tk.Label(cell, text=label, bg=KEY, fg=TEXT, font=("Segoe UI", 24, "bold"))
            big.pack(expand=True, pady=(8, 0))
            small = tk.Label(cell, text=caption, bg=KEY, fg=MUTED, font=("Segoe UI", 9))
            small.pack(pady=(0, 8))
            self.key_widgets[key] = (cell, big, small)
            for w in (cell, big, small):
                w.bind("<ButtonPress-1>", lambda e, k=key: self._press(k))
                w.bind("<ButtonRelease-1>", lambda e, k=key: self._release(k))

        ttk.Label(pad, text="Hold W A S D to drive  -  release to stop", style="Muted.TLabel").pack(pady=(14, 0))

    # ---- input handling ----
    def _bind_keyboard(self):
        def keyname(event):
            return event.keysym.lower()

        def on_press(event):
            k = keyname(event)
            if k in KEYS:
                self._press(k)

        def on_release(event):
            k = keyname(event)
            if k in KEYS:
                self._release(k)

        self.root.bind("<KeyPress>", on_press)
        self.root.bind("<KeyRelease>", on_release)

    def _press(self, key):
        if key in self.held:
            return  # key auto-repeat
        self.held.append(key)
        self._refresh_drive()

    def _release(self, key):
        if key in self.held:
            self.held.remove(key)
            self._refresh_drive()

    def _refresh_drive(self):
        for k, (cell, big, small) in self.key_widgets.items():
            on = k in self.held
            bg = KEY_ACTIVE if on else KEY
            cell.config(bg=bg)
            big.config(bg=bg, fg="white" if on else TEXT)
            small.config(bg=bg, fg="white" if on else MUTED)
        # Most recently pressed key wins; releasing falls back to any key still held
        self.send_command(KEYS[self.held[-1]][0] if self.held else 'S')

    # ---- serial ----
    def scan_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
        else:
            self.port_combo.set("No ports found")

    def _set_status(self, text, color):
        self.status_label.config(text=text)
        self.status_dot.itemconfig(self.dot, fill=color)

    def _disconnect(self, send_stop=True):
        conn, self.serial_conn = self.serial_conn, None
        if conn:
            try:
                if send_stop and conn.is_open:
                    conn.write(b'S')
            except (serial.SerialException, OSError):
                pass
            try:
                conn.close()
            except (serial.SerialException, OSError):
                pass
        self.connect_btn.config(text="Connect", state="normal")
        self._set_status("Disconnected", BAD)

    def toggle_connection(self):
        if self.serial_conn and self.serial_conn.is_open:
            self._disconnect()
            return

        selected = self.port_combo.get()
        if not selected or selected == "No ports found":
            messagebox.showwarning("No Port", "Please select a valid COM port first.")
            return

        port_name = selected.split(" - ")[0]
        try:
            self.serial_conn = serial.Serial(port_name, 9600, timeout=1, write_timeout=1)
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Could not open {port_name}:\n{e}")
            return

        # Wait for Arduino auto-reset on serial open without freezing the UI
        self.connect_btn.config(state="disabled")
        self._set_status(f"Connecting to {port_name}...", WARN)
        self.root.after(2000, lambda: self._finish_connect(port_name))

    def _finish_connect(self, port_name):
        if not (self.serial_conn and self.serial_conn.is_open):
            return
        self.connect_btn.config(text="Disconnect", state="normal")
        self._set_status(f"Connected to {port_name}", OK)

    def send_command(self, char_cmd):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(char_cmd.encode('utf-8'))
            except (serial.SerialException, OSError):
                self._disconnect(send_stop=False)
                messagebox.showerror("Connection Lost", "Serial connection was lost.")

    def on_closing(self):
        self._disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = CarController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
