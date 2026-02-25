#!/usr/bin/env python3
"""
Herramienta de calibración de motores para robot diferencial.
Identifica el orden de los motores, cuáles están invertidos,
y genera un archivo de configuración.

Uso: ros2 run pwm_controller motor_calibrator
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import threading

PWM_MIN = 1100
PWM_MID = 1500
PWM_MAX = 1900
PWM_TEST_FWD = 1600  # small forward thrust
PWM_TEST_REV = 1400  # small reverse thrust

MOTOR_LABELS = ["M1", "M2", "M3", "M4"]

POSITIONS = [
    "Front Left",
    "Front Right",
    "Rear Left",
    "Rear Right"
]

COLORS = {
    'bg':        '#0a0e1a',
    'panel':     '#111827',
    'border':    '#1e3a5f',
    'accent':    '#00d4ff',
    'accent2':   '#ff6b35',
    'success':   '#00ff88',
    'warning':   '#ffcc00',
    'danger':    '#ff3366',
    'text':      '#e2e8f0',
    'text_dim':  '#64748b',
    'btn':       '#1e3a5f',
    'btn_hover': '#2d5a8e',
}


class MotorCalibratorNode(Node):
    def __init__(self):
        super().__init__('motor_calibrator')
        self.pub = self.create_publisher(UInt16MultiArray, '/pwm_outputs', 10)
        self.current_pwm = [PWM_MID, PWM_MID, PWM_MID, PWM_MID]
        # Publicar neutro continuamente
        self.create_timer(0.05, self.publish_pwm)

    def publish_pwm(self):
        msg = UInt16MultiArray()
        msg.data = self.current_pwm
        self.pub.publish(msg)

    def set_motor(self, motor_idx, value):
        self.current_pwm[motor_idx] = int(value)

    def set_all_neutral(self):
        self.current_pwm = [PWM_MID, PWM_MID, PWM_MID, PWM_MID]


class CalibratorGUI:
    def __init__(self, node: MotorCalibratorNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Motor Calibrator — BlueRobotics T200")
        self.root.configure(bg=COLORS['bg'])
        self.root.geometry("900x780")
        self.root.resizable(False, False)

        # Estado de calibración
        self.motor_positions = {}   # motor_idx -> position string
        self.motor_inverted  = {}   # motor_idx -> bool
        self.current_test_motor = None

        # Variables tkinter
        self.position_vars  = [tk.StringVar(value="— Unassigned —") for _ in range(4)]
        self.inverted_vars  = [tk.BooleanVar(value=False) for _ in range(4)]
        self.slider_vars    = [tk.IntVar(value=PWM_MID) for _ in range(4)]

        self._build_ui()

    def _build_ui(self):
        self._header()
        self._motor_panels()
        self._footer()

    def _header(self):
        hdr = tk.Frame(self.root, bg=COLORS['bg'])
        hdr.pack(fill='x', padx=30, pady=(24, 0))

        tk.Label(hdr,
                 text="MOTOR CALIBRATOR",
                 font=("Courier New", 22, "bold"),
                 fg=COLORS['accent'],
                 bg=COLORS['bg']).pack(side='left')

        tk.Label(hdr,
                 text="T200 · BlueRobotics · Differential Drive",
                 font=("Courier New", 10),
                 fg=COLORS['text_dim'],
                 bg=COLORS['bg']).pack(side='left', padx=(16, 0), pady=(8, 0))

        # línea separadora
        sep = tk.Frame(self.root, bg=COLORS['border'], height=1)
        sep.pack(fill='x', padx=30, pady=(12, 0))

        tk.Label(self.root,
                 text="① Use sliders to test each motor  ② Assign its position  ③ Check if inverted  ④ Export config",
                 font=("Courier New", 9),
                 fg=COLORS['text_dim'],
                 bg=COLORS['bg']).pack(pady=(8, 4))

    def _motor_panels(self):
        container = tk.Frame(self.root, bg=COLORS['bg'])
        container.pack(fill='both', expand=True, padx=24, pady=8)

        for i in range(4):
            self._motor_card(container, i, i % 2, i // 2)

    def _motor_card(self, parent, idx, col, row):
        card = tk.Frame(parent,
                        bg=COLORS['panel'],
                        highlightbackground=COLORS['border'],
                        highlightthickness=1)
        card.grid(row=row, column=col, padx=8, pady=8, sticky='nsew')
        parent.grid_columnconfigure(col, weight=1)
        parent.grid_rowconfigure(row, weight=1)

        # Título motor
        title_frame = tk.Frame(card, bg=COLORS['accent'], height=3)
        title_frame.pack(fill='x')

        tk.Label(card,
                 text=MOTOR_LABELS[idx],
                 font=("Courier New", 13, "bold"),
                 fg=COLORS['accent'],
                 bg=COLORS['panel']).pack(pady=(12, 2))

        # Valor PWM actual
        self.pwm_label = tk.Label(card,
                                   text=f"PWM: {PWM_MID} µs",
                                   font=("Courier New", 10),
                                   fg=COLORS['text_dim'],
                                   bg=COLORS['panel'])
        self.pwm_label.pack()
        # guardar referencia
        card._pwm_label = self.pwm_label

        # Slider
        slider = tk.Scale(card,
                          from_=PWM_MIN,
                          to=PWM_MAX,
                          orient='horizontal',
                          variable=self.slider_vars[idx],
                          command=lambda v, i=idx, c=card: self._on_slider(i, v, c),
                          bg=COLORS['panel'],
                          fg=COLORS['panel'],
                          troughcolor=COLORS['btn'],
                          activebackground=COLORS['accent'],
                          highlightthickness=0,
                          length=340,
                          showvalue=False)
        slider.pack(padx=16, pady=4)

        # Botones rápidos
        btn_frame = tk.Frame(card, bg=COLORS['panel'])
        btn_frame.pack(pady=4)

        self._btn(btn_frame, "◀ REV", lambda i=idx: self._quick(i, PWM_TEST_REV), COLORS['danger']).pack(side='left', padx=4)
        self._btn(btn_frame, "■ STOP", lambda i=idx: self._quick(i, PWM_MID), COLORS['text_dim']).pack(side='left', padx=4)
        self._btn(btn_frame, "FWD ▶", lambda i=idx: self._quick(i, PWM_TEST_FWD), COLORS['success']).pack(side='left', padx=4)

        # Separador
        tk.Frame(card, bg=COLORS['border'], height=1).pack(fill='x', padx=16, pady=8)

        # Posición
        tk.Label(card,
                 text="POSITION:",
                 font=("Courier New", 9, "bold"),
                 fg=COLORS['text_dim'],
                 bg=COLORS['panel']).pack(anchor='w', padx=16)

        pos_menu = ttk.Combobox(card,
                                textvariable=self.position_vars[idx],
                                values=["— Unassigned —"] + POSITIONS,
                                state='readonly',
                                font=("Courier New", 10),
                                width=24)
        pos_menu.pack(padx=16, pady=4)

        # Estilo combobox
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TCombobox',
                        fieldbackground=COLORS['btn'],
                        background=COLORS['btn'],
                        foreground=COLORS['text'],
                        selectbackground=COLORS['btn'],
                        selectforeground=COLORS['text'])
        style.map('TCombobox',
                  fieldbackground=[('readonly', COLORS['btn'])],
                  foreground=[('readonly', COLORS['text'])],
                  selectbackground=[('readonly', COLORS['btn'])],
                  selectforeground=[('readonly', COLORS['text'])])

        # Invertido
        inv_frame = tk.Frame(card, bg=COLORS['panel'])
        inv_frame.pack(pady=4)

        tk.Checkbutton(inv_frame,
                       text="MOTOR INVERTED",
                       variable=self.inverted_vars[idx],
                       font=("Courier New", 10, "bold"),
                       fg=COLORS['warning'],
                       bg=COLORS['panel'],
                       activebackground=COLORS['panel'],
                       activeforeground=COLORS['warning'],
                       selectcolor=COLORS['bg'],
                       cursor='hand2').pack()

    def _btn(self, parent, text, cmd, color):
        b = tk.Button(parent,
                      text=text,
                      command=cmd,
                      font=("Courier New", 9, "bold"),
                      fg=color,
                      bg=COLORS['btn'],
                      activebackground=COLORS['btn_hover'],
                      activeforeground=color,
                      relief='flat',
                      padx=10,
                      pady=4,
                      cursor='hand2',
                      bd=0)
        return b

    def _on_slider(self, idx, value, card):
        v = int(value)
        self.node.set_motor(idx, v)
        card._pwm_label.config(text=f"PWM: {v} µs")

    def _quick(self, idx, value):
        self.slider_vars[idx].set(value)
        self.node.set_motor(idx, value)

    def _footer(self):
        sep = tk.Frame(self.root, bg=COLORS['border'], height=1)
        sep.pack(fill='x', padx=30, pady=(4, 8))

        btn_row = tk.Frame(self.root, bg=COLORS['bg'])
        btn_row.pack(pady=(0, 8))

        tk.Button(btn_row,
                  text="■  ALL NEUTRAL",
                  command=self._all_neutral,
                  font=("Courier New", 11, "bold"),
                  fg=COLORS['text'],
                  bg=COLORS['btn'],
                  activebackground=COLORS['btn_hover'],
                  relief='flat',
                  padx=20, pady=8,
                  cursor='hand2').pack(side='left', padx=8)

        tk.Button(btn_row,
                  text="✔  EXPORT CONFIGURATION",
                  command=self._export,
                  font=("Courier New", 11, "bold"),
                  fg=COLORS['bg'],
                  bg=COLORS['accent'],
                  activebackground=COLORS['success'],
                  relief='flat',
                  padx=20, pady=8,
                  cursor='hand2').pack(side='left', padx=8)

        # Output box
        self.output_frame = tk.Frame(self.root, bg=COLORS['bg'])
        self.output_frame.pack(fill='x', padx=30, pady=(0, 16))

        self.output_text = tk.Text(self.output_frame,
                                   height=6,
                                   font=("Courier New", 9),
                                   bg=COLORS['panel'],
                                   fg=COLORS['success'],
                                   insertbackground=COLORS['accent'],
                                   relief='flat',
                                   padx=12, pady=8,
                                   wrap='none')
        self.output_text.pack(fill='x')
        self.output_text.insert('end', "# Exported configuration will appear here — copy and paste into controller_node.py\n")
        self.output_text.config(state='disabled')

    def _all_neutral(self):
        for i in range(4):
            self.slider_vars[i].set(PWM_MID)
        self.node.set_all_neutral()

    def _export(self):
        # Validar que todos tengan posición asignada
        positions = [v.get() for v in self.position_vars]
        if "— Unassigned —" in positions:
            messagebox.showerror("Error", "Assign a position to all motors before exporting.")
            return

        assigned = [p for p in positions if p != "— Unassigned —"]
        if len(set(assigned)) != len(assigned):
            messagebox.showerror("Error", "Duplicate positions found. Each motor must have a unique position.")
            return

        inverted = [v.get() for v in self.inverted_vars]

        # Construir mapping
        # motor_map[posicion] = (motor_idx, invertido)
        motor_map = {}
        for i in range(4):
            motor_map[positions[i]] = {
                'motor': i + 1,
                'pin': [12, 14, 27, 25][i],
                'inverted': inverted[i]
            }

        # Generar el bloque de código para pegar en controller_node.py
        config_block = self._generate_config(motor_map, positions, inverted)

        # Guardar archivo
        output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motor_config.json')
        config_data = {
            'motors': [
                {
                    'index': i,
                    'motor_label': MOTOR_LABELS[i],
                    'pin': [12, 14, 27, 25][i],
                    'position': positions[i],
                    'inverted': inverted[i]
                }
                for i in range(4)
            ],
            'motor_map': motor_map
        }
        with open(output_path, 'w') as f:
            json.dump(config_data, f, indent=2)

        # Mostrar en el text box
        self.output_text.config(state='normal')
        self.output_text.delete('1.0', 'end')
        self.output_text.insert('end', config_block)
        self.output_text.config(state='disabled')

        messagebox.showinfo("✔ Exported",
                            f"Configuration saved to:\n{output_path}\n\n"
                            "Copy the code block from the text box and paste it into controller_node.py")

    def _generate_config(self, motor_map, positions, inverted):
        # Encontrar índices para cada posición
        def idx_for(pos):
            for i, p in enumerate(positions):
                if p == pos:
                    return i
            return -1

        fl = idx_for("Front Left")
        fr = idx_for("Front Right")
        bl = idx_for("Rear Left")
        br = idx_for("Rear Right")

        inv_list = [str(inverted[i]).lower() for i in range(4)]

        lines = [
            "# ─── MOTOR CONFIGURATION (generated by motor_calibrator) ────────────────────",
            "# Paste this block into controller_node.py replacing the equivalent section",
            "",
            "# Indices in /pwm_outputs array: [M1, M2, M3, M4]",
            "",
            f"MOTOR_FL = {fl}   # Front Left  → {'INVERTED' if inverted[fl] else 'normal'}",
            f"MOTOR_FR = {fr}   # Front Right → {'INVERTED' if inverted[fr] else 'normal'}",
            f"MOTOR_BL = {bl}   # Rear Left   → {'INVERTED' if inverted[bl] else 'normal'}",
            f"MOTOR_BR = {br}   # Rear Right  → {'INVERTED' if inverted[br] else 'normal'}",
            "",
            f"MOTOR_INVERTED = [{inv_list[0]}, {inv_list[1]}, {inv_list[2]}, {inv_list[3]}]",
            "",
            "# Usage in control_loop():",
            "# pwm[MOTOR_FL] = self.apply_inversion(MOTOR_FL, left)",
            "# pwm[MOTOR_FR] = self.apply_inversion(MOTOR_FR, right)",
            "# pwm[MOTOR_BL] = self.apply_inversion(MOTOR_BL, left)",
            "# pwm[MOTOR_BR] = self.apply_inversion(MOTOR_BR, right)",
        ]
        return "\n".join(lines)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.node.set_all_neutral()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCalibratorNode()

    # ROS2 spin en hilo separado
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # GUI en hilo principal
    gui = CalibratorGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
