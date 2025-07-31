import sys
import threading
import subprocess
import shutil
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext

import psutil

from gui.ConfigData import load_saved_config, save_config_to_disk, ConfigData
from CarlaControl import kill_carla_processes, start_carla


class TextRedirector:
    def __init__(self, log_fn):
        self.log = log_fn

    def write(self, txt):
        for line in txt.rstrip().splitlines():
            self.log(line)

    def flush(self):
        pass


class ManualControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("CARLA Manual Control")
        root.geometry("800x600")
        for c in range(4):
            root.columnconfigure(c, weight=1)
        for r in range(7):
            root.rowconfigure(r, weight=1 if r == 6 else 0)

        # Load existing config
        self.config: ConfigData = load_saved_config()

        # UI vars
        self.exe_var = tk.StringVar(value=self.config.carla_executable)
        self.input_var = tk.StringVar(value=self.config.input_path)
        self.output_var = tk.StringVar(value=self.config.output_path)
        self.filename_var = tk.StringVar(value=self.config.new_file_name)

        # Layout (entries + buttons) unchanged …
        tk.Label(root, text="CARLA Executable Path").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        tk.Entry(root, textvariable=self.exe_var).grid(row=0, column=1, columnspan=2, sticky="ew", padx=5)
        tk.Button(root, text="Browse", command=self.browse_exe).grid(row=0, column=3, padx=5)

        tk.Label(root, text="Default Recordings Folder").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        tk.Entry(root, textvariable=self.input_var).grid(row=1, column=1, columnspan=2, sticky="ew", padx=5)
        tk.Button(root, text="Browse", command=self.browse_input).grid(row=1, column=3, padx=5)

        tk.Label(root, text="New Output Folder").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        tk.Entry(root, textvariable=self.output_var).grid(row=2, column=1, columnspan=2, sticky="ew", padx=5)
        tk.Button(root, text="Browse", command=self.browse_output).grid(row=2, column=3, padx=5)

        tk.Label(root, text="New File Name").grid(row=3, column=0, sticky="w", padx=5, pady=5)
        tk.Entry(root, textvariable=self.filename_var).grid(row=3, column=1, columnspan=3, sticky="ew", padx=5)

        # Action buttons
        self.start_btn = tk.Button(root, text="Start", command=self.on_start)
        self.transform_btn = tk.Button(root, text="Transform & Move Latest",
                                       command=self.on_transform,
                                       state="disabled")  # start disabled until MC starts
        self.stop_btn = tk.Button(root, text="Stop", command=self.on_stop,
                                  state="disabled")

        self.start_btn.grid(row=4, column=1, sticky="ew", pady=10, padx=5)
        self.transform_btn.grid(row=4, column=2, sticky="ew", pady=10, padx=5)
        self.stop_btn.grid(row=4, column=3, sticky="ew", pady=10, padx=5)

        # Log area
        self.log_text = scrolledtext.ScrolledText(root, state="disabled")
        self.log_text.grid(row=6, column=0, columnspan=4, sticky="nsew", padx=5, pady=5)

        # Cancellation event
        self.cancel_event = threading.Event()

        # Process handles
        self._mc_proc = None
        self._mc_pid = None

    # ---- Browsers ----
    def browse_exe(self):
        path = filedialog.askopenfilename(
            title="Select CARLA Executable",
            filetypes=[("Executable", "*.exe;*.sh"), ("All files", "*.*")]
        )
        if path:
            self.exe_var.set(path)

    def browse_input(self):
        path = filedialog.askdirectory(title="Select default recordings folder")
        if path:
            self.input_var.set(path)

    def browse_output(self):
        path = filedialog.askdirectory(title="Select output folder")
        if path:
            self.output_var.set(path)

    def log(self, msg: str):
        def _append():
            self.log_text.configure(state='normal')
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.see(tk.END)
            self.log_text.configure(state='disabled')

        self.root.after(0, _append)

    def save_config(self):
        self.config.carla_executable = self.exe_var.get().strip()
        self.config.input_path = self.input_var.get().strip()
        self.config.output_path = self.output_var.get().strip()
        self.config.new_file_name = self.filename_var.get().strip()
        save_config_to_disk(self.config)

    def clear_log(self):
        self.log_text.configure(state='normal')
        self.log_text.delete('1.0', tk.END)
        self.log_text.configure(state='disabled')

    # ---- Start workflow ----
    def on_start(self):
        self.clear_log()
        # reset cancel flag
        self.cancel_event.clear()

        # validate & save...
        if not all([self.exe_var.get(), self.input_var.get(), self.output_var.get()]):
            messagebox.showerror("Missing Fields", "CARLA exe, input & output folders are required.")
            return

        self.save_config()
        self.start_btn.config(state="disabled")
        # Transform remains disabled until MC actually starts
        self.transform_btn.config(state="disabled")
        # Stop is enabled and stays enabled
        self.stop_btn.config(state="normal")

        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state="disabled")

        threading.Thread(target=self._run_start, args=(self.config,), daemon=True).start()

    def _run_start(self, cfg: ConfigData):
        # 1) Kill existing Carla
        self.log(">> Killing existing CARLA processes…")
        kill_carla_processes()
        self.log(">> Existing CARLA processes killed.")
        if self.cancel_event.is_set():
            return

        # 2) Start Carla
        self.log(f">> Starting CARLA: {cfg.carla_executable}")
        start_carla(cfg.carla_executable)

        # 3) Wait 20 s
        self.log(">> Waiting 20 seconds for CARLA to initialize…")
        threading.Event().wait(20)
        if self.cancel_event.is_set():
            return

        # 4) Launch manual_control.py
        base = Path(cfg.carla_executable).parent
        mc_py = base / "PythonAPI" / "examples" / "manual_control.py"
        if not mc_py.exists():
            self.log(f"!! manual_control.py not found at {mc_py}")
        elif not self.cancel_event.is_set():
            cmd = [sys.executable, str(mc_py)]
            self.log(f">> Launching {mc_py.name}")
            self._mc_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True  # <— cross-platform: new process group/session
            )
            self._mc_pid = self._mc_proc.pid

            # now MC is running → enable Transform
            self.root.after(0, lambda: self.transform_btn.config(state="normal"))

            # stream its output, but exit if canceled
            for line in self._mc_proc.stdout:
                if self.cancel_event.is_set():
                    break
                self.log(line.rstrip())

        self.log(">> manual_control.py exited")
        # re-enable Start (but not Stop; Stop stays enabled until user clicks it)
        self.root.after(0, lambda: self.start_btn.config(state="normal"))

    # ---- Transform & Move workflow ----
    def on_transform(self):
        fn = self.filename_var.get().strip()
        if not fn:
            messagebox.showerror("Missing Field", "Please enter a new file name.")
            return

        self.save_config()
        threading.Thread(target=self._run_transform, args=(self.config, fn), daemon=True).start()

    def _run_transform(self, cfg: ConfigData, fn: str):
        inp = Path(cfg.input_path)
        out = Path(cfg.output_path)

        recs = list(inp.glob("*.rec"))
        if not recs:
            self.log("!! No .rec files found.")
        else:
            latest = max(recs, key=lambda p: p.stat().st_mtime)
            # New format: Year_Month_Day_Hour_Minute
            ts = datetime.now().strftime("%Y_%m_%d_%H_%M")
            dst = out / f"{fn}_{ts}.rec"
            self.log(f">> Moving {latest.name} → {dst.name}")
            try:
                shutil.move(str(latest), str(dst))
                self.log(">> Move successful.")
            except Exception as e:
                self.log(f"!! Error moving file: {e}")

    def on_stop(self):
        # signal cancellation in your workflow thread
        self.cancel_event.set()
        self.log(">> Stop requested: killing manual_control and CARLA.")

        # kill manual_control.py and all its children
        if self._mc_proc:
            try:
                proc = psutil.Process(self._mc_proc.pid)
                # kill all children
                for child in proc.children(recursive=True):
                    self.log(f">> Killing child pid {child.pid}")
                    child.kill()
                # kill the parent itself
                self.log(f">> Killing manual_control.py pid {proc.pid}")
                proc.kill()
            except psutil.NoSuchProcess:
                pass
            except Exception as e:
                self.log(f"!! Error killing manual_control tree: {e}")
            finally:
                self._mc_proc = None
                self._mc_pid = None

        # then kill CARLA instances
        kill_carla_processes()
        self.log(">> CARLA processes killed.")

        # reset buttons
        self.start_btn.config(state="normal")
        self.transform_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.log(">> DONE.")


if __name__ == "__main__":
    root = tk.Tk()
    ManualControlApp(root)
    root.mainloop()
