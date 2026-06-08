"""
Tkinter ROS 2 GUI for 4WD manual motor control.

Normal drive buttons publish /cmd_vel. Individual wheel buttons publish
/wheel_test_command in the Arduino sketch format: "FL 100", "RR -80", "STOP".

The window is resizable and theme-styled; driving is also available from the
keyboard (arrows / WASD to move, Space to stop, Esc for emergency stop).
"""

from __future__ import annotations

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


WHEEL_CIRCUMFERENCE_CM = 2.0 * 3.141592653589793 * 19.0
WHEELS = ("FL", "FR", "RL", "RR")

# Palet — koyu tema, yüksek kontrast
COLORS = {
    "bg": "#1e2430",
    "panel": "#2a3242",
    "fg": "#e6e9ef",
    "muted": "#9aa4b2",
    "accent": "#4f8cff",
    "go": "#2faa5a",
    "go_active": "#37c469",
    "warn": "#e08a2b",
    "warn_active": "#f29b39",
    "danger": "#d6453d",
    "danger_active": "#e85a52",
    "wheel": "#3a4357",
    "wheel_active": "#4a5670",
}


class MotorControlGui(Node):
    def __init__(self) -> None:
        super().__init__("motor_control_gui")

        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("default_speed_ratio", 0.35)
        self.declare_parameter("default_wheel_rpm", 80)

        self._max_linear = float(self.get_parameter("max_linear_speed").value)
        self._max_angular = float(self.get_parameter("max_angular_speed").value)
        default_speed = float(self.get_parameter("default_speed_ratio").value)
        default_rpm = int(self.get_parameter("default_wheel_rpm").value)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._wheel_pub = self.create_publisher(String, "/wheel_test_command", 10)
        self.create_subscription(Float32MultiArray, "/wheel_rpm", self._wheel_rpm_cb, 10)

        self._enabled = False
        self._active_motion = "stop"
        self._set_rpm = {wheel: 0.0 for wheel in WHEELS}
        self._measured_rpm = {wheel: 0.0 for wheel in WHEELS}

        self._root = tk.Tk()
        self._root.title("4WD ROS Motor Control")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.configure(bg=COLORS["bg"])
        self._root.minsize(420, 620)

        self._speed = tk.DoubleVar(value=default_speed)
        self._speed_label = tk.StringVar(value=f"{default_speed:.2f}")
        self._wheel_rpm = tk.IntVar(value=default_rpm)
        self._status = tk.StringVar(value="Hazir - Baslat kapali")

        self._setup_style()
        self._build_ui()
        self._bind_keys()
        self._root.after(100, self._tick)

    def run(self) -> None:
        self._root.mainloop()

    # ------------------------------------------------------------------ style
    def _setup_style(self) -> None:
        style = ttk.Style(self._root)
        style.theme_use("clam")

        base = tkfont.nametofont("TkDefaultFont")
        base.configure(size=10)
        self._title_font = tkfont.Font(family=base.actual("family"), size=12, weight="bold")

        style.configure(".", background=COLORS["bg"], foreground=COLORS["fg"])
        style.configure("TFrame", background=COLORS["bg"])
        style.configure("Card.TLabelframe", background=COLORS["panel"], borderwidth=0,
                        relief="flat")
        style.configure("Card.TLabelframe.Label", background=COLORS["panel"],
                        foreground=COLORS["accent"], font=self._title_font)
        style.configure("TLabel", background=COLORS["panel"], foreground=COLORS["fg"])
        style.configure("Status.TLabel", background=COLORS["bg"], foreground=COLORS["muted"])
        style.configure("Muted.TLabel", background=COLORS["panel"], foreground=COLORS["muted"])
        style.configure("Value.TLabel", background=COLORS["panel"], foreground=COLORS["accent"],
                        font=self._title_font)

        style.configure("TScale", background=COLORS["panel"])
        style.configure("TSpinbox", fieldbackground=COLORS["bg"], foreground=COLORS["fg"],
                        arrowcolor=COLORS["fg"])

        def button(name: str, bg: str, active: str, fg: str = "#ffffff") -> None:
            style.configure(name, background=bg, foreground=fg, borderwidth=0,
                            focuscolor=bg, padding=(8, 7), font=base)
            style.map(name,
                      background=[("active", active), ("pressed", active)],
                      foreground=[("disabled", COLORS["muted"])])

        button("Go.TButton", COLORS["go"], COLORS["go_active"])
        button("Warn.TButton", COLORS["warn"], COLORS["warn_active"])
        button("Danger.TButton", COLORS["danger"], COLORS["danger_active"])
        button("Drive.TButton", COLORS["accent"], "#6ba0ff")
        button("Wheel.TButton", COLORS["wheel"], COLORS["wheel_active"])

        # Telemetri tablosu
        style.configure("Telemetry.Treeview", background=COLORS["bg"],
                        fieldbackground=COLORS["bg"], foreground=COLORS["fg"],
                        borderwidth=0, rowheight=24)
        style.configure("Telemetry.Treeview.Heading", background=COLORS["panel"],
                        foreground=COLORS["accent"], borderwidth=0, font=base)
        style.map("Telemetry.Treeview.Heading", background=[("active", COLORS["panel"])])

    # --------------------------------------------------------------------- ui
    def _build_ui(self) -> None:
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)

        main = ttk.Frame(self._root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        # Bölümler dikeyde ölçeklensin
        main.rowconfigure(0, weight=2)   # genel sürüş
        main.rowconfigure(1, weight=2)   # tek teker
        main.rowconfigure(2, weight=3)   # telemetri
        main.rowconfigure(3, weight=0)   # durum çubuğu

        self._build_drive_section(main)
        self._build_wheel_section(main)
        self._build_telemetry_section(main)

        status = ttk.Label(main, textvariable=self._status, style="Status.TLabel",
                           anchor="w", padding=(2, 8))
        status.grid(row=3, column=0, sticky="ew")

    def _build_drive_section(self, parent: ttk.Frame) -> None:
        sec = ttk.Labelframe(parent, text=" Genel Surus  (/cmd_vel) ",
                             style="Card.TLabelframe", padding=12)
        sec.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        for col in range(3):
            sec.columnconfigure(col, weight=1)

        ttk.Button(sec, text="Baslat", style="Go.TButton",
                   command=self._enable).grid(row=0, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(sec, text="Dur", style="Warn.TButton",
                   command=self._stop_all).grid(row=0, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(sec, text="Acil STOP", style="Danger.TButton",
                   command=self._emergency_stop).grid(row=0, column=2, sticky="ew", padx=3, pady=3)

        ttk.Button(sec, text="▲ Ileri", style="Drive.TButton",
                   command=lambda: self._set_motion("forward")).grid(row=1, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(sec, text="◀ Sol", style="Drive.TButton",
                   command=lambda: self._set_motion("left")).grid(row=2, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(sec, text="■ Dur", style="Warn.TButton",
                   command=self._stop_motion).grid(row=2, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(sec, text="Sag ▶", style="Drive.TButton",
                   command=lambda: self._set_motion("right")).grid(row=2, column=2, sticky="ew", padx=3, pady=3)
        ttk.Button(sec, text="▼ Geri", style="Drive.TButton",
                   command=lambda: self._set_motion("backward")).grid(row=3, column=1, sticky="ew", padx=3, pady=3)

        ttk.Label(sec, text="Hiz Orani", style="Muted.TLabel").grid(
            row=4, column=0, sticky="w", pady=(12, 0))
        ttk.Scale(sec, from_=0.0, to=1.0, variable=self._speed, orient="horizontal",
                  command=self._on_speed_change).grid(
            row=4, column=1, sticky="ew", pady=(12, 0), padx=3)
        ttk.Label(sec, textvariable=self._speed_label, style="Value.TLabel", anchor="e").grid(
            row=4, column=2, sticky="e", pady=(12, 0))

    def _build_wheel_section(self, parent: ttk.Frame) -> None:
        sec = ttk.Labelframe(parent, text=" Tek Teker Testi  (/wheel_test_command) ",
                             style="Card.TLabelframe", padding=12)
        sec.grid(row=1, column=0, sticky="nsew", pady=8)
        for col in range(4):
            sec.columnconfigure(col, weight=1)

        ttk.Label(sec, text="RPM", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(sec, from_=0, to=300, textvariable=self._wheel_rpm, width=8).grid(
            row=0, column=1, sticky="w", padx=3, pady=3)
        ttk.Button(sec, text="Teker STOP", style="Warn.TButton",
                   command=self._wheel_stop).grid(row=0, column=2, columnspan=2, sticky="ew", padx=3, pady=3)

        for wheel, row, col in [("FL", 1, 0), ("FR", 1, 2), ("RL", 2, 0), ("RR", 2, 2)]:
            ttk.Button(sec, text=f"{wheel} +", style="Wheel.TButton",
                       command=lambda w=wheel: self._wheel_cmd(w, 1)).grid(
                row=row, column=col, sticky="ew", padx=3, pady=3)
            ttk.Button(sec, text=f"{wheel} −", style="Wheel.TButton",
                       command=lambda w=wheel: self._wheel_cmd(w, -1)).grid(
                row=row, column=col + 1, sticky="ew", padx=3, pady=3)

    def _build_telemetry_section(self, parent: ttk.Frame) -> None:
        sec = ttk.Labelframe(parent, text=" Teker Hiz Gosterge Paneli ",
                             style="Card.TLabelframe", padding=12)
        sec.grid(row=2, column=0, sticky="nsew", pady=(8, 4))
        sec.columnconfigure(0, weight=1)
        sec.rowconfigure(0, weight=1)

        columns = ("set_rpm", "measured_rpm", "measured_speed")
        tree = ttk.Treeview(sec, columns=columns, show="tree headings",
                            style="Telemetry.Treeview", height=4, selectmode="none")
        tree.heading("#0", text="Teker")
        tree.heading("set_rpm", text="Set RPM")
        tree.heading("measured_rpm", text="Olculen RPM")
        tree.heading("measured_speed", text="Olculen cm/s")
        tree.column("#0", width=70, anchor="w", stretch=True)
        for col in columns:
            tree.column(col, width=100, anchor="center", stretch=True)
        for wheel in WHEELS:
            tree.insert("", "end", iid=wheel, text=wheel,
                        values=("0.0", "0.0", "0.0"))
        tree.grid(row=0, column=0, sticky="nsew")
        self._tree = tree

    # ----------------------------------------------------------------- keys
    def _bind_keys(self) -> None:
        bindings = {
            "<Up>": "forward", "<w>": "forward", "<W>": "forward",
            "<Down>": "backward", "<s>": "backward", "<S>": "backward",
            "<Left>": "left", "<a>": "left", "<A>": "left",
            "<Right>": "right", "<d>": "right", "<D>": "right",
        }
        for key, motion in bindings.items():
            self._root.bind(key, lambda _e, m=motion: self._key_motion(m))
        self._root.bind("<space>", lambda _e: self._stop_motion())
        self._root.bind("<Escape>", lambda _e: self._emergency_stop())

    def _key_motion(self, motion: str) -> None:
        # Klavye auto-repeat'inde gereksiz yeniden komut göndermeyi engelle
        if self._enabled and self._active_motion == motion:
            return
        self._set_motion(motion)

    def _on_speed_change(self, _value: str) -> None:
        self._speed_label.set(f"{float(self._speed.get()):.2f}")

    # ----------------------------------------------------------------- logic
    def _enable(self) -> None:
        self._enabled = True
        self._status.set("Baslat acik - genel surus aktif")

    def _set_motion(self, motion: str) -> None:
        self._wheel_stop()
        self._enabled = True
        self._active_motion = motion
        self._publish_motion()

    def _stop_motion(self) -> None:
        self._active_motion = "stop"
        self._publish_motion()
        self._status.set("Genel surus durdu")

    def _stop_all(self) -> None:
        self._enabled = False
        self._active_motion = "stop"
        self._set_all_rpm(0.0)
        self._publish_motion()
        self._wheel_stop()
        self._status.set("Dur - tum komutlar sifirlandi")

    def _emergency_stop(self) -> None:
        self._stop_all()
        self._status.set("ACIL STOP gonderildi")

    def _wheel_cmd(self, wheel: str, sign: int) -> None:
        self._enabled = False
        self._active_motion = "stop"
        self._publish_motion()
        rpm = max(0, min(300, int(self._wheel_rpm.get()))) * sign
        msg = String()
        msg.data = f"{wheel} {rpm}"
        self._wheel_pub.publish(msg)
        self._set_all_rpm(0.0)
        self._set_rpm[wheel] = float(rpm)
        self._refresh_telemetry_labels()
        self._status.set(f"Tek teker test: {msg.data}")

    def _wheel_stop(self) -> None:
        msg = String()
        msg.data = "STOP"
        self._wheel_pub.publish(msg)
        self._set_all_rpm(0.0)
        self._refresh_telemetry_labels()

    def _publish_motion(self) -> None:
        msg = Twist()
        if self._enabled:
            ratio = max(0.0, min(1.0, float(self._speed.get())))
            if self._active_motion == "forward":
                msg.linear.x = self._max_linear * ratio
            elif self._active_motion == "backward":
                msg.linear.x = -self._max_linear * ratio
            elif self._active_motion == "left":
                msg.angular.z = self._max_angular * ratio
            elif self._active_motion == "right":
                msg.angular.z = -self._max_angular * ratio
        self._cmd_pub.publish(msg)
        self._update_drive_setpoints(msg)

    def _update_drive_setpoints(self, msg: Twist) -> None:
        if abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6:
            self._set_all_rpm(0.0)
            self._refresh_telemetry_labels()
            return

        left_v = msg.linear.x - (msg.angular.z * 0.34 / 2.0)
        right_v = msg.linear.x + (msg.angular.z * 0.34 / 2.0)
        left_rpm = left_v * 60.0 / (WHEEL_CIRCUMFERENCE_CM / 100.0)
        right_rpm = right_v * 60.0 / (WHEEL_CIRCUMFERENCE_CM / 100.0)

        self._set_rpm["FL"] = left_rpm
        self._set_rpm["RL"] = left_rpm
        self._set_rpm["FR"] = right_rpm
        self._set_rpm["RR"] = right_rpm
        self._refresh_telemetry_labels()

    def _set_all_rpm(self, rpm: float) -> None:
        for wheel in WHEELS:
            self._set_rpm[wheel] = rpm

    def _wheel_rpm_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        for wheel, rpm in zip(WHEELS, msg.data[:4]):
            self._measured_rpm[wheel] = float(rpm)
        self._refresh_telemetry_labels()

    def _refresh_telemetry_labels(self) -> None:
        if not hasattr(self, "_tree"):
            return
        for wheel in WHEELS:
            measured = self._measured_rpm[wheel]
            cmps = measured * WHEEL_CIRCUMFERENCE_CM / 60.0
            self._tree.item(wheel, values=(
                f"{self._set_rpm[wheel]:.1f}",
                f"{measured:.1f}",
                f"{cmps:.1f}",
            ))

    def _tick(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)
        self._publish_motion()
        self._root.after(150, self._tick)

    def _on_close(self) -> None:
        self._stop_all()
        self._root.after(100, self._root.destroy)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorControlGui()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
