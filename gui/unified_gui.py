#!/usr/bin/env python3
import sys, tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

from gui.config_data import Config, load, save
from gui.workers import (
    ManualControlWorker, MoveLatestRecWorker,
    TransformWorker, RecordVideoWorker, CarlaServerWorker
)


# ──────────────────────────────────────────────────────────────────────────────
class UnifiedCarlaGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CARLA unified GUI")
        self.geometry("900x700")
        self.resizable(True, True)

        # ── shared config vars ────────────────────────────────────────────────
        self.cfg: Config = load()
        self.exe_var = tk.StringVar(value=self.cfg.carla_executable)
        self.in_var = tk.StringVar(value=self.cfg.input_path)
        self.out_var = tk.StringVar(value=self.cfg.output_path)
        self.defrec_var = tk.StringVar(value=self.cfg.default_recordings_folder)
        self.ext_var = tk.StringVar(value=self.cfg.recording_extension)
        self.name_var = tk.StringVar(value=self.cfg.new_file_name)

        self.vw_var = tk.IntVar(value=self.cfg.video_width)
        self.vh_var = tk.IntVar(value=self.cfg.video_height)
        self.vid_var = tk.IntVar(value=self.cfg.vehicle_id)
        self.bbox_var = tk.BooleanVar(value=self.cfg.with_bboxes)

        self._active_worker = None  # type: _ThreadWorker | None

        # ── build UI ──────────────────────────────────────────────────────────
        nb = ttk.Notebook(self);
        nb.pack(fill="both", expand=True)
        self._tab_manual(nb)
        self._tab_transform(nb)
        self._tab_video(nb)

        # log pane
        self.log = scrolledtext.ScrolledText(self, height=10, state="disabled")
        self.log.pack(fill="both", expand=False, padx=4, pady=4)

        self._redirect_console()
        self._setup_autosave()

    # ════════════════════════════════════════════════════════════════════════
    #                    tab construction helpers
    # ════════════════════════════════════════════════════════════════════════
    def _entry_row(self, parent, label, var, browse=None, width=45):
        row = tk.Frame(parent);
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label, width=22, anchor="w").pack(side="left")
        e = tk.Entry(row, textvariable=var, width=width);
        e.pack(side="left", fill="x", expand=True)
        if browse:
            tk.Button(row, text="…", command=browse).pack(side="left", padx=2)

    # ── MANUAL DRIVE TAB ────────────────────────────────────────────────────
    def _tab_manual(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="Manual Drive")
        tk.Label(f, text="Kill/start CARLA, run manual_control.py, then optionally\n"
                         "move the newest *.rec into your recordings folder.").pack(pady=5)

        # inputs
        self._entry_row(f, "CARLA executable:", self.exe_var,
                        lambda: self._filepick(self.exe_var))
        self._entry_row(f, "Default recordings folder:", self.defrec_var,
                        lambda: self._dirpick(self.defrec_var))
        self._entry_row(f, "New file-name prefix:", self.name_var)

        # actions
        self.carla_btn = tk.Button(f, text="Start CARLA server", width=25,
                                   command=self._toggle_carla)
        self.carla_btn.pack(pady=4)

        self.start_btn = tk.Button(f, text="Start manual driving", width=25,
                                   command=self._start_manual)
        self.start_btn.pack(pady=8)

        self.move_btn = tk.Button(f, text="Move latest *.rec",
                                  command=self._move_latest, state="disabled")
        self.move_btn.pack(pady=2)

        self.stop_btn = tk.Button(f, text="Stop", command=self._stop_worker,
                                  state="disabled")
        self.stop_btn.pack(pady=8)

    # ── TRANSFORM TAB ───────────────────────────────────────────────────────
    def _tab_transform(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="Transform")
        tk.Label(f, text="Replay a .rec in CARLA and let helpers.CarlaMonitor\n"
                         "write processed data into an output file.").pack(pady=5)

        self._entry_row(f, "CARLA executable:", self.exe_var,
                        lambda: self._filepick(self.exe_var))
        self._entry_row(f, "Input recording / folder:", self.in_var,
                        lambda: self._anypick(self.in_var))
        self._entry_row(f, "Output folder:", self.out_var,
                        lambda: self._dirpick(self.out_var))

        tk.Button(f, text="Start transform", command=self._start_transform).pack(pady=10)

    # ── VIDEO TAB ───────────────────────────────────────────────────────────
    def _tab_video(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="Record ➜ MP4")
        tk.Label(f, text="Export a recording directly to an mp4 stream.\n"
                         "Bounding-box overlay is optional.").pack(pady=5)

        self._entry_row(f, "CARLA executable:", self.exe_var,
                        lambda: self._filepick(self.exe_var))
        self._entry_row(f, "Input recording / folder:", self.in_var,
                        lambda: self._anypick(self.in_var))
        self._entry_row(f, "Output folder:", self.out_var,
                        lambda: self._dirpick(self.out_var))

        # video-specific
        vid_frame = ttk.LabelFrame(f, text="Video parameters");
        vid_frame.pack(fill="x", padx=4, pady=6)
        self._entry_row(vid_frame, "Width:", self.vw_var, width=8)
        self._entry_row(vid_frame, "Height:", self.vh_var, width=8)
        self._entry_row(vid_frame, "Vehicle ID (-1 = ego):", self.vid_var, width=8)
        tk.Checkbutton(vid_frame, text="Draw 3-D bounding boxes",
                       variable=self.bbox_var).pack(anchor="w", padx=4, pady=4)

        tk.Button(f, text="Start recording", command=self._start_video).pack(pady=10)

    # ════════════════════════════════════════════════════════════════════════
    #              orchestration: attach workers, validation
    # ════════════════════════════════════════════════════════════════════════
    def _attach_worker(self, worker, after_start=lambda: None):
        if worker.exclusive and getattr(self, "_active_worker", None):
            return messagebox.showwarning("Busy", "Another exclusive task is running.")

        worker.start()
        if worker.exclusive:
            self._active_worker = worker

        def poll():
            if worker.is_alive():
                self.after(500, poll)
            else:
                self._active_worker = None
                self.stop_btn.config(state="disabled")
                self.move_btn.config(state="disabled")

        poll()

    def _stop_worker(self):
        if self._active_worker:
            self._active_worker.cancel()
        if getattr(self, "_active_worker", None):
            self._active_worker.cancel()
            self._active_worker = None
        if hasattr(self, "_carla_worker") and self._carla_worker.is_alive():
            self._carla_worker.cancel()
            self.carla_btn.config(text="Start CARLA server")

    # ---------------- CARLA server only -------------------------------------
    def _toggle_carla(self):
        if hasattr(self, "_carla_worker") and self._carla_worker.is_alive():
            # Stop
            self._carla_worker.cancel()
            self.carla_btn.config(text="Start CARLA server")
        else:
            if not self._validate(["exe"]): return
            self._carla_worker = CarlaServerWorker(self._collect_cfg(), self._log)
            self._carla_worker.start()
            self.carla_btn.config(text="Stop CARLA server")

    # ---------------- manual-drive workflow ---------------------------------
    def _start_manual(self):
        if not self._validate(["exe", "defrec"]): return
        self._attach_worker(
            ManualControlWorker(self._collect_cfg(), self._log),
            after_start=lambda: self.move_btn.config(state="normal")
        )

    def _move_latest(self):
        if not self.name_var.get().strip():
            return messagebox.showerror("Missing", "File-name prefix required.")
        self._attach_worker(
            MoveLatestRecWorker(self._collect_cfg(), self.name_var.get(), self._log)
        )

    # ---------------- transform --------------------------------------------
    def _start_transform(self):
        if not self._validate(["exe", "in", "out"]): return
        self._attach_worker(TransformWorker(self._collect_cfg(), self._log))

    # ---------------- video -------------------------------------------------
    def _start_video(self):
        if not self._validate(["exe", "in", "out"]): return
        cfg = self._collect_cfg()
        cfg.video_width = self.vw_var.get()
        cfg.video_height = self.vh_var.get()
        cfg.vehicle_id = self.vid_var.get()
        cfg.with_bboxes = self.bbox_var.get()
        self._attach_worker(RecordVideoWorker(cfg, self._log))

    # ════════════════════════════════════════════════════════════════════════
    #                             utils & misc
    # ════════════════════════════════════════════════════════════════════════
    def _setup_autosave(self):
        """Attach trace() callbacks so that *any* edit writes the JSON config."""
        for var in (
                self.exe_var, self.in_var, self.out_var, self.defrec_var,
                self.ext_var, self.name_var, self.vw_var, self.vh_var,
                self.vid_var, self.bbox_var
        ):
            var.trace_add("write", self._auto_save)

    def _auto_save(self, *args):
        self._collect_cfg()  # updates self.cfg and persist to disk

    def _validate(self, needed):
        need_msg = {
            "exe": ("CARLA executable", self.exe_var),
            "in": ("Input path/folder", self.in_var),
            "out": ("Output folder", self.out_var),
            "defrec": ("Default recordings folder", self.defrec_var)
        }
        for key in needed:
            label, var = need_msg[key]
            if not var.get().strip():
                messagebox.showerror("Missing", f"{label} required.")
                return False
        return True

    def _collect_cfg(self) -> Config:
        c = self.cfg
        c.carla_executable = self.exe_var.get().strip()
        c.input_path = self.in_var.get().strip()
        c.output_path = self.out_var.get().strip()
        c.default_recordings_folder = self.defrec_var.get().strip()
        c.recording_extension = self.ext_var.get().strip()
        c.new_file_name = self.name_var.get().strip()
        save(c)
        return c

    # redirect print()/tracebacks into the log pane
    def _redirect_console(self):
        class _R:
            def __init__(self, gui): self.gui = gui

            def write(self, txt):
                for l in txt.rstrip().splitlines():
                    self.gui._log(l)

            def flush(self): pass

        sys.stdout = sys.stderr = _R(self)

    def _log(self, txt):
        self.log.configure(state="normal")
        self.log.insert("end", txt + "\n");
        self.log.see("end")
        self.log.configure(state="disabled")

    # pickers
    def _filepick(self, var):
        p = filedialog.askopenfilename();
        var.set(p or var.get())

    def _dirpick(self, var):
        p = filedialog.askdirectory();
        var.set(p or var.get())

    def _anypick(self, var):
        p = filedialog.askopenfilename() or filedialog.askdirectory();
        var.set(p or var.get())


if __name__ == "__main__":
    UnifiedCarlaGUI().mainloop()
