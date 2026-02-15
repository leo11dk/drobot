import subprocess
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

from ament_index_python.packages import get_package_share_directory

from drobot_bringup.worldgen import write_world, default_output_path


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Drobot World Launcher")
        self.root.geometry("720x420")

        # state
        self.mode = tk.StringVar(value="controller")
        self.num_obs = tk.IntVar(value=10)
        self.seed = tk.StringVar(value="")  # optional
        self.proc = None  # launch process handle

        # compute generated world directory in INSTALL share of drobot_description
        desc_share = Path(get_package_share_directory("drobot_description"))
        self.generated_dir = desc_share / "worlds" / "generated"

        self._build_ui()

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=12)
        frm.pack(fill="both", expand=True)

        ttk.Label(frm, text="Mode").grid(row=0, column=0, sticky="w", pady=(0, 6))
        mode_combo = ttk.Combobox(frm, textvariable=self.mode, values=["controller", "nav2_2d", "nav2_3d"], state="readonly")
        mode_combo.grid(row=0, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(frm, text="Obstacles (boxes)").grid(row=1, column=0, sticky="w", pady=(0, 6))
        ttk.Spinbox(frm, from_=0, to=200, textvariable=self.num_obs).grid(row=1, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(frm, text="Seed (optional)").grid(row=2, column=0, sticky="w", pady=(0, 6))
        ttk.Entry(frm, textvariable=self.seed).grid(row=2, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(frm, text="Generated world path (install share)").grid(row=3, column=0, sticky="w", pady=(12, 6))
        self.world_path_lbl = ttk.Label(frm, text=str(self.generated_dir), wraplength=520)
        self.world_path_lbl.grid(row=3, column=1, sticky="w", pady=(12, 6))

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(18, 0))
        ttk.Button(btns, text="Generate + Launch", command=self.on_launch).pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Button(btns, text="Stop", command=self.on_stop).pack(side="left", fill="x", expand=True)

        frm.grid_columnconfigure(1, weight=1)

    def on_launch(self):
        # 1) Generate world
        try:
            n = int(self.num_obs.get())
            seed_txt = self.seed.get().strip()
            seed_val = int(seed_txt) if seed_txt else None

            out_path = default_output_path(self.generated_dir)
            out_path = write_world(out_path, num_obstacles=n, seed=seed_val)
        except Exception as e:
            messagebox.showerror("Generate failed", str(e))
            return

        # 2) Map mode -> boolean flags for launch file
        mode = self.mode.get()
        use_controller = "true" if mode == "controller" else "false"
        use_nav2_2d = "true" if mode == "nav2_2d" else "false"
        use_nav2_3d = "true" if mode == "nav2_3d" else "false"

        # 3) Stop previous run if any
        self.on_stop()

        # 4) Launch orchestrator (assumes this UI was run from a sourced environment via ros2 run)
        cmd = [
            "ros2", "launch", "drobot_bringup", "bringup.launch.py",
            f"world:={str(out_path)}",
            f"mode:={mode}",
            f"use_controller:={use_controller}",
            f"use_nav2_2d:={use_nav2_2d}",
            f"use_nav2_3d:={use_nav2_3d}",
        ]

        try:
            self.proc = subprocess.Popen(cmd)
        except FileNotFoundError:
            messagebox.showerror(
                "ros2 not found",
                "ros2 command not found.\nRun this UI using:\nsource /opt/ros/jazzy/setup.bash\nsource ~/Desktop/drobot/install/setup.bash\nros2 run drobot_bringup world_ui"
            )
            return

        messagebox.showinfo("Launched", f"World generated:\n{out_path}\n\nLaunching mode: {mode}")

    def on_stop(self):
        if self.proc and (self.proc.poll() is None):
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.proc = None


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()
