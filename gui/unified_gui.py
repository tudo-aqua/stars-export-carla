#!/usr/bin/env python3
"""
Unified CARLA GUI – rev-4
•  Separate output folders for Transform / Video
•  Uses new config keys: transformer_output_path, video_output_path
"""
from __future__ import annotations
import sys, tkinter as tk
from pathlib import Path
from tkinter import ttk, filedialog, messagebox, scrolledtext

from gui.config_data import Config, load, save
from gui.workers import (
    ManualControlWorker, MoveLatestRecWorker,
    TransformWorker, RecordVideoWorker, CarlaServerWorker,
    kill_carla
)


class UnifiedCarlaGUI(tk.Tk):
    # ───────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__()
        self.title("CARLA unified GUI")
        self.geometry("950x800")
        self.resizable(True, True)

        # ── load persisted settings ────────────────────────────────────
        self.cfg: Config = load()

        self.exe_var = tk.StringVar(value=self.cfg.carla_executable)
        self.ext_var = tk.StringVar(value=self.cfg.recording_extension)

        # manual-drive
        self.man_out_var = tk.StringVar(value=self.cfg.manual_output_dir)
        self.defrec_var = tk.StringVar(value=self.cfg.default_recordings_folder)
        self.name_var = tk.StringVar(value=self.cfg.new_file_name)

        # transform / video I/O
        self.tr_in_var = tk.StringVar(value=self.cfg.transform_input_file)
        self.tr_out_var = tk.StringVar(value=self.cfg.transformer_output_path)
        self.vid_in_var = tk.StringVar(value=self.cfg.video_input_file)
        self.vid_out_var = tk.StringVar(value=self.cfg.video_output_path)

        # video opts
        self.vw_var = tk.IntVar(value=self.cfg.video_width)
        self.vh_var = tk.IntVar(value=self.cfg.video_height)
        self.vid_var = tk.IntVar(value=self.cfg.vehicle_id)
        self.bbox_var = tk.BooleanVar(value=self.cfg.with_bboxes)
        self.begin_var = tk.DoubleVar(value=self.cfg.begin_at)
        end_default = -1 if self.cfg.end_at == float("inf") else self.cfg.end_at
        self.end_var = tk.DoubleVar(value=end_default)

        self._active_worker = None
        self._carla_worker = None

        # ── build notebook ─────────────────────────────────────────────
        nb = ttk.Notebook(self);
        nb.pack(fill="both", expand=True)
        self._tab_server(nb)
        self._tab_manual(nb)
        self._tab_transform(nb)
        self._tab_video(nb)

        # log pane
        self.log = scrolledtext.ScrolledText(self, height=10, state="disabled")
        self.log.pack(fill="both", expand=False, padx=4, pady=4)

        self._redirect_console()
        self._setup_autosave()

    # ═══════════════════════════════════════════════════════════════════
    #                     tab construction helpers
    # ═══════════════════════════════════════════════════════════════════
    def _entry_row(self, parent, label, var, browse=None, width=45):
        row = tk.Frame(parent);
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label, width=26, anchor="w").pack(side="left")
        tk.Entry(row, textvariable=var, width=width) \
            .pack(side="left", fill="x", expand=True)
        if browse:
            tk.Button(row, text="…", command=browse) \
                .pack(side="left", padx=2)

    # ─ server tab ─
    def _tab_server(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="CARLA Server")
        tk.Label(f, text="Start or stop a head-less CARLA instance.").pack(pady=5)
        self._entry_row(f, "CARLA executable:", self.exe_var,
                        lambda: self._filepick(self.exe_var))
        self.server_btn = tk.Button(f, text="Start CARLA server",
                                    width=25, command=self._toggle_carla)
        self.server_btn.pack(pady=10)

    # ─ manual-drive tab ─
    def _tab_manual(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="Manual Drive")
        tk.Label(f, text="Kill → start CARLA → run manual_control.py.").pack(pady=5)

        self._entry_row(f, "CARLA executable:", self.exe_var,
                        lambda: self._filepick(self.exe_var))
        self._entry_row(f, "Recording extension:", self.ext_var)
        self._entry_row(f, "CARLA output folder:", self.man_out_var,
                        lambda: self._dirpick(self.man_out_var))
        self._entry_row(f, "Archive recordings folder:", self.defrec_var,
                        lambda: self._dirpick(self.defrec_var))
        self._entry_row(f, "New file-name prefix:", self.name_var)

        self.start_btn = tk.Button(f, text="Start manual driving",
                                   width=25, command=self._start_manual)
        self.start_btn.pack(pady=8)

        self.move_btn = tk.Button(f, text="Move 'manual_recording'",
                                  command=self._move_latest, state="disabled")
        self.move_btn.pack(pady=2)

        self.stop_btn = tk.Button(f, text="Stop",
                                  command=self._stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)

    # ─ transform tab ─
    def _tab_transform(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="Transform")
        tk.Label(f, text="Replay a recording and dump processed data.").pack(pady=5)

        self._entry_row(f, "Recording extension:", self.ext_var)
        self._entry_row(f, "Input recording:", self.tr_in_var,
                        lambda: self._recfilepick(self.tr_in_var))
        self._entry_row(f, "Output folder:", self.tr_out_var,
                        lambda: self._dirpick(self.tr_out_var))

        tk.Button(f, text="Start transform",
                  command=self._start_transform).pack(pady=10)

    # ─ video tab ─
    def _tab_video(self, nb):
        f = ttk.Frame(nb);
        nb.add(f, text="Record ➜ MP4")
        tk.Label(f, text="Export a recording directly to mp4.").pack(pady=5)

        self._entry_row(f, "Recording extension:", self.ext_var)
        self._entry_row(f, "Input recording:", self.vid_in_var,
                        lambda: self._recfilepick(self.vid_in_var))
        self._entry_row(f, "Output folder:", self.vid_out_var,
                        lambda: self._dirpick(self.vid_out_var))

        vp = ttk.LabelFrame(f, text="Video parameters");
        vp.pack(fill="x", padx=4, pady=6)
        self._entry_row(vp, "Width:", self.vw_var, width=8)
        self._entry_row(vp, "Height:", self.vh_var, width=8)
        self._entry_row(vp, "Vehicle ID (-1 = ego):", self.vid_var, width=8)
        self._entry_row(vp, "Start at (s):", self.begin_var, width=8)
        self._entry_row(vp, "End at (s, -1 = file end):", self.end_var, width=8)
        tk.Checkbutton(vp, text="Draw 3-D bounding boxes",
                       variable=self.bbox_var).pack(anchor="w", padx=4, pady=4)

        tk.Button(f, text="Start recording",
                  command=self._start_video).pack(pady=10)

    # ═══════════════════════════════════════════════════════════════════
    #                worker orchestration / validation
    # ═══════════════════════════════════════════════════════════════════
    # ── helper to launch a background worker ───────────────────────────
    def _attach_worker(self, worker, *, enable_move: bool = False):
        if worker.exclusive and self._active_worker:
            return messagebox.showwarning("Busy", "Another exclusive task is running.")

        worker.start()

        # keep track of the 1 exclusive task that may run at a time
        if worker.exclusive:
            self._active_worker = worker
            self.stop_btn.config(state="normal")

        # optionally make “Move recording” available
        if enable_move:
            self.move_btn.config(state="normal")

        # poll until *this* worker finishes
        def poll():
            if worker.is_alive():
                self.after(500, poll)
            else:
                # only reset Stop-button if the exclusive task has ended
                if worker is self._active_worker:
                    self._active_worker = None
                    self.stop_btn.config(state="disabled")

                # only turn off Move-button when the *manual-drive* workflow ends
                from gui.workers import ManualControlWorker
                if isinstance(worker, ManualControlWorker):
                    self.move_btn.config(state="disabled")

        poll()

    # ─── start helpers ────────────────────────────────────────────────
    def _toggle_carla(self):
        if self._carla_worker and self._carla_worker.is_alive():
            self._carla_worker.cancel()
            self.server_btn.config(text="Start CARLA server")
        else:
            if not self._validate([("CARLA executable", self.exe_var)]):
                return
            self._carla_worker = CarlaServerWorker(self._collect_cfg(), self._log)
            self._carla_worker.start()
            self.server_btn.config(text="Stop CARLA server")

    def _start_manual(self):
        need = [("CARLA executable", self.exe_var),
                ("CARLA output folder", self.man_out_var)]
        if not self._validate(need): return
        self._attach_worker(ManualControlWorker(self._collect_cfg(), self._log),
                            enable_move=True)

    def _move_latest(self):
        if not self.name_var.get().strip():
            return messagebox.showerror("Missing", "File-name prefix required.")
        self._attach_worker(MoveLatestRecWorker(self._collect_cfg(),
                                                self.name_var.get(), self._log))

    def _start_transform(self):
        need = [("CARLA executable", self.exe_var),
                ("Input recording", self.tr_in_var),
                ("Output folder", self.tr_out_var)]
        if not self._validate(need): return

        cfg = self._collect_cfg()
        cfg.transform_input_file = self.tr_in_var.get().strip()
        cfg.transformer_output_path = self.tr_out_var.get().strip()
        self._attach_worker(TransformWorker(cfg, self._log))

    def _start_video(self):
        need = [("CARLA executable", self.exe_var),
                ("Input recording", self.vid_in_var),
                ("Output folder", self.vid_out_var)]
        if not self._validate(need): return

        cfg = self._collect_cfg()
        cfg.video_input_file = self.vid_in_var.get().strip()
        cfg.video_output_path = self.vid_out_var.get().strip()

        cfg.video_width = self.vw_var.get()
        cfg.video_height = self.vh_var.get()
        cfg.vehicle_id = self.vid_var.get()
        cfg.with_bboxes = self.bbox_var.get()

        cfg.begin_at = max(0.0, self.begin_var.get())
        end_val = self.end_var.get()
        cfg.end_at = float("inf") if end_val < 0 else end_val

        self._attach_worker(RecordVideoWorker(cfg, self._log))

    def _stop_worker(self):
        if self._active_worker:
            self._active_worker.cancel()
        if self._carla_worker and self._carla_worker.is_alive():
            self._carla_worker.cancel()
            self.server_btn.config(text="Start CARLA server")
        kill_carla()

    # ═══════════════════════════════════════════════════════════════════
    #                      persistence helpers
    # ═══════════════════════════════════════════════════════════════════
    def _setup_autosave(self):
        for var in (
                self.exe_var, self.ext_var,
                self.man_out_var, self.defrec_var, self.name_var,
                self.tr_in_var, self.tr_out_var,
                self.vid_in_var, self.vid_out_var,
                self.vw_var, self.vh_var, self.vid_var, self.bbox_var,
                self.begin_var, self.end_var
        ):
            var.trace_add("write", self._auto_save)

    def _auto_save(self, *_):
        self._collect_cfg()

    def _collect_cfg(self) -> Config:
        c = self.cfg
        c.carla_executable = self.exe_var.get().strip()
        c.recording_extension = self._norm_ext(self.ext_var.get())
        c.manual_output_dir = self.man_out_var.get().strip()
        c.default_recordings_folder = self.defrec_var.get().strip()
        c.new_file_name = self.name_var.get().strip()
        c.transform_input_file = self.tr_in_var.get().strip()
        c.transformer_output_path = self.tr_out_var.get().strip()
        c.video_input_file = self.vid_in_var.get().strip()
        c.video_output_path = self.vid_out_var.get().strip()
        c.with_bboxes = self.bbox_var.get()

        c.begin_at = max(0.0, self.begin_var.get())
        end_val = self.end_var.get()
        c.end_at = float("inf") if end_val < 0 else end_val
        save(c)
        return c

    # ── utilities ──────────────────────────────────────────────────────
    def _validate(self, pairs):
        for label, var in pairs:
            if not var.get().strip():
                messagebox.showerror("Missing", f"{label} required.")
                return False
        return True

    def _filepick(self, var):
        p = filedialog.askopenfilename();
        var.set(p or var.get())

    def _recfilepick(self, var):
        ext = self._norm_ext(self.ext_var.get())
        p = filedialog.askopenfilename(filetypes=[(f"{ext} files", f"*{ext}"),
                                                  ("All files", "*.*")])
        var.set(p or var.get())

    def _dirpick(self, var):
        p = filedialog.askdirectory();
        var.set(p or var.get())

    @staticmethod
    def _norm_ext(ext: str) -> str:
        return ext if ext.startswith(".") else f".{ext}"

    # redirect stdout/stderr into GUI
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


if __name__ == "__main__":
    UnifiedCarlaGUI().mainloop()
