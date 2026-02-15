import threading
import tkinter as tk

import rclpy
from rclpy.node import Node


class UiNode(Node):
    def __init__(self, root: tk.Tk, stop_event: threading.Event):
        super().__init__('drobot_ui')
        self.root = root
        self.stop_event = stop_event

        self.root.title("Drobot UI")
        self.root.geometry("420x220")

        label = tk.Label(root, text="Drobot UI (tkinter + rclpy)", font=("Arial", 14))
        label.pack(pady=12)

        tk.Button(root, text="Ping ROS logger", command=self.on_ping).pack(pady=6)
        tk.Button(root, text="Quit", command=self.on_quit).pack(pady=6)

        # If user closes window via the X button
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

        self.get_logger().info("UI started")

    def on_ping(self):
        self.get_logger().info("Ping button pressed")

    def on_quit(self):
        self.get_logger().info("UI quitting")
        self.stop_event.set()
        # exit tkinter loop cleanly
        self.root.quit()


def _spin(node: Node, stop_event: threading.Event):
    # spin_once loop so we can exit promptly when stop_event is set
    while rclpy.ok() and (not stop_event.is_set()):
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)

    stop_event = threading.Event()
    root = tk.Tk()
    node = UiNode(root, stop_event)

    spin_thread = threading.Thread(target=_spin, args=(node, stop_event), daemon=True)
    spin_thread.start()

    try:
        root.mainloop()
    finally:
        # Ensure ROS stops before destroying node/process exits
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        try:
            root.destroy()
        except Exception:
            pass
