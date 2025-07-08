#!/usr/bin/env python3

import tkinter as tk
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

STICK_RADIUS = 60
STICK_INNER_RADIUS = 44
PUBLISH_HZ = 10

PALETTE = {
    "bg_main": "#444444",
    "bg_panel": "#555555",
    "stick_fill": "#666666",
    "abxy": {
        "A": "#4CAF50",
        "B": "#E57373",
        "X": "#64B5F6",
        "Y": "#E0B800"  # 약간 더 어두운 노란색
    },
    "abxy_hover": {
        "A": "#81C784",
        "B": "#EF9A9A",
        "X": "#90CAF9",
        "Y": "#FFEB7D"
    },
    "abxy_hover_fg": "#222222"
}

class VirtualJoystickNode(Node):
    def __init__(self):
        super().__init__('virtual_joystick_node')
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 11
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_joy)

    def publish_joy(self):
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.joy_msg)

    def press_button(self, idx):
        self.joy_msg.buttons[idx] = 1

    def release_button(self, idx):
        self.joy_msg.buttons[idx] = 0

    def update_axes(self, stick, direction):
        if stick == "L":
            if direction == "UP":
                self.joy_msg.axes[1] = 1.0
            elif direction == "DOWN":
                self.joy_msg.axes[1] = -1.0
            elif direction == "LEFT":
                self.joy_msg.axes[0] = -1.0
            elif direction == "RIGHT":
                self.joy_msg.axes[0] = 1.0
        elif stick == "R":
            if direction == "UP":
                self.joy_msg.axes[4] = 1.0
            elif direction == "DOWN":
                self.joy_msg.axes[4] = -1.0
            elif direction == "LEFT":
                self.joy_msg.axes[3] = -1.0
            elif direction == "RIGHT":
                self.joy_msg.axes[3] = 1.0

    def reset_axes(self):
        for i in [0,1,3,4,6,7]:
            self.joy_msg.axes[i] = 0.0

    def update_dpad(self, direction):
        if direction == "UP":
            self.joy_msg.axes[7] = 1.0
        elif direction == "DOWN":
            self.joy_msg.axes[7] = -1.0
        elif direction == "LEFT":
            self.joy_msg.axes[6] = -1.0
        elif direction == "RIGHT":
            self.joy_msg.axes[6] = 1.0
        elif direction == "CENTER":
            self.reset_dpad()

    def reset_dpad(self):
        self.joy_msg.axes[6] = 0.0
        self.joy_msg.axes[7] = 0.0

class CircularStick(tk.Canvas):
    def __init__(self, master, label, callback):
        super().__init__(master, width=160, height=160, bg=PALETTE["bg_main"], highlightthickness=0, bd=0)
        self.label = label
        self.callback = callback
        self.center = (80, 80)
        self.stick_radius = STICK_RADIUS
        self.create_oval(0, 0, 160, 160, fill=PALETTE["bg_panel"], outline='white', width=2)
        self.stick = self.create_oval(80-STICK_INNER_RADIUS, 80-STICK_INNER_RADIUS,
                                      80+STICK_INNER_RADIUS, 80+STICK_INNER_RADIUS,
                                      fill=PALETTE["stick_fill"], outline="")
        self.bind("<B1-Motion>", self.move_stick)
        self.bind("<ButtonRelease-1>", self.reset_stick)

    def move_stick(self, event):
        dx = event.x - self.center[0]
        dy = event.y - self.center[1]
        dist = math.hypot(dx, dy)

        max_movement = (self.stick_radius - STICK_INNER_RADIUS / 1.2) * 2  # 두 배로 확대
        if dist > max_movement:
            dx = dx / dist * max_movement
            dy = dy / dist * max_movement

        self.coords(self.stick,
                    self.center[0]+dx-STICK_INNER_RADIUS, self.center[1]+dy-STICK_INNER_RADIUS,
                    self.center[0]+dx+STICK_INNER_RADIUS, self.center[1]+dy+STICK_INNER_RADIUS)

        angle = math.degrees(math.atan2(-dy, dx))
        direction = self.angle_to_direction(angle)
        if direction:
            self.callback(self.label, direction)

    def reset_stick(self, event=None):
        self.coords(self.stick, 80-STICK_INNER_RADIUS, 80-STICK_INNER_RADIUS, 80+STICK_INNER_RADIUS, 80+STICK_INNER_RADIUS)
        self.callback(self.label, "CENTER")

    def angle_to_direction(self, angle):
        if -45 <= angle < 45:
            return "RIGHT"
        elif 45 <= angle < 135:
            return "UP"
        elif angle >= 135 or angle < -135:
            return "LEFT"
        elif -135 <= angle < -45:
            return "DOWN"
        return "CENTER"

# ABXY 버튼 hover 효과 함수

def add_hover_effect(button, key):
    def on_enter(e):
        button.config(bg=PALETTE["abxy_hover"][key], fg=PALETTE["abxy_hover_fg"])
    def on_leave(e):
        button.config(bg=PALETTE["abxy"][key], fg="white")
    button.bind("<Enter>", on_enter)
    button.bind("<Leave>", on_leave)

class JoystickGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.configure(bg=PALETTE["bg_main"])
        self.root.title("🎮 Xbox Controller GUI Layout")

        main = tk.Frame(self.root, bg=PALETTE["bg_main"])
        main.pack(padx=20, pady=20)

        # Left Panel
        left = tk.Frame(main, bg=PALETTE["bg_main"])
        left.grid(row=0, column=0, padx=20)
        triggers_l = tk.Frame(left, bg=PALETTE["bg_main"])
        triggers_l.pack()
        self.add_button(triggers_l, "LT", 2).pack(side=tk.LEFT, padx=2)
        self.add_button(triggers_l, "LB", 4).pack(side=tk.LEFT, padx=2)
        self.l_stick = CircularStick(left, "L", self.on_stick_input)
        self.l_stick.pack(pady=10)
        dpad = tk.Frame(left, bg=PALETTE["bg_main"])
        dpad.pack(pady=10)
        for r, c, label, idx in [(0,1,"↑",7), (1,0,"←",6), (1,1,"•",6), (1,2,"→",6), (2,1,"↓",7)]:
            self.add_button(dpad, label, idx).grid(row=r, column=c)

        # Center Buttons
        center = tk.Frame(main, bg=PALETTE["bg_main"])
        center.grid(row=0, column=1, padx=20)
        self.add_button(center, "Back", 6).pack(side=tk.LEFT, padx=5)
        self.add_button(center, "Guide", 8).pack(side=tk.LEFT, padx=5)
        self.add_button(center, "Start", 7).pack(side=tk.LEFT, padx=5)

        # Right Panel
        right = tk.Frame(main, bg=PALETTE["bg_main"])
        right.grid(row=0, column=2, padx=20)
        triggers_r = tk.Frame(right, bg=PALETTE["bg_main"])
        triggers_r.pack()
        self.add_button(triggers_r, "RB", 5).pack(side=tk.LEFT, padx=2)
        self.add_button(triggers_r, "RT", 5).pack(side=tk.LEFT, padx=2)
        abxy = tk.Frame(right, bg=PALETTE["bg_main"])
        abxy.pack(pady=10)
        for r, c, label, idx in [(0,1,"Y",3), (1,0,"X",2), (1,2,"B",1), (2,1,"A",0)]:
            self.add_button(abxy, label, idx, color=PALETTE["abxy"][label], shape="circle").grid(row=r, column=c)
        self.r_stick = CircularStick(right, "R", self.on_stick_input)
        self.r_stick.pack(pady=10)

    def add_button(self, parent, label, btn_index, color=None, shape=None):
        size = 50
        canvas = tk.Canvas(parent, width=size, height=size, bg=PALETTE["bg_main"], highlightthickness=0)
        if shape == "circle":
            canvas.create_oval(2, 2, size-2, size-2, fill=color if color else PALETTE["bg_panel"], outline="white")
            canvas.create_text(size/2, size/2, text=label, fill="white", font=("Arial", 10, "bold"))
            canvas.bind("<ButtonPress-1>", lambda e: self.ros_node.press_button(btn_index))
            canvas.bind("<ButtonRelease-1>", lambda e: self.ros_node.release_button(btn_index))
            if label in PALETTE["abxy"]:
                add_hover_effect(canvas, label)
            return canvas
        else:
            btn = tk.Button(parent, text=label, width=4, height=2, bg=color if color else PALETTE["bg_panel"], fg="white")
            btn.bind("<ButtonPress>", lambda e: self.ros_node.press_button(btn_index))
            btn.bind("<ButtonRelease>", lambda e: self.ros_node.release_button(btn_index))
            return btn

        canvas.bind("<ButtonPress-1>", lambda e: self.ros_node.press_button(btn_index))
        canvas.bind("<ButtonRelease-1>", lambda e: self.ros_node.release_button(btn_index))
        return canvas

    def on_stick_input(self, label, direction):
        self.ros_node.reset_axes()
        if direction != "CENTER":
            self.ros_node.update_axes(label, direction)

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    node = VirtualJoystickNode()
    gui = JoystickGUI(node)

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0.01)
        gui.root.after(10, spin_ros)

    gui.root.after(10, spin_ros)
    gui.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
