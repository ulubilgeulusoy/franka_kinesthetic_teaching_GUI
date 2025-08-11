#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# --------- CONFIG (edit if needed) ---------
ROS_SETUP = "source ~/franka_ws/install/setup.bash"
ROBOT_IP = "172.16.0.2"  # change if needed
# -------------------------------------------

def bash_cmd(cmd: str):
    return f'bash -lc "{ROS_SETUP}; {cmd}"'

class ProcessGroup:
    def __init__(self, line_queue: queue.Queue):
        self.procs = []
        self._lock = threading.Lock()
        self.line_queue = line_queue

    def start(self, cmd: str):
        p = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1,
            universal_newlines=True,
        )
        with self._lock:
            self.procs.append(p)

        threading.Thread(target=self._pump_output, args=(p,), daemon=True).start()
        return p

    def _pump_output(self, p: subprocess.Popen):
        try:
            for line in iter(p.stdout.readline, ""):
                self.line_queue.put(line.rstrip("\n"))
        finally:
            if p.stdout:
                p.stdout.close()
            p.wait()
            self.line_queue.put(f"[exit {p.pid}] code={p.returncode}")

    def is_alive(self):
        with self._lock:
            return any(p.poll() is None for p in self.procs)

    def terminate_all(self, sig=signal.SIGINT):
        with self._lock:
            for p in self.procs:
                if p.poll() is None:
                    try:
                        os.killpg(os.getpgid(p.pid), sig)
                    except ProcessLookupError:
                        pass

    def clear_finished(self):
        with self._lock:
            self.procs = [p for p in self.procs if p.poll() is None]

class FR3TeachRunGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FR3 Teach & Run (thread-safe)")
        self.geometry("900x600")

        self.line_queue = queue.Queue()
        self.teach_pg = ProcessGroup(self.line_queue)
        self.run_pg = ProcessGroup(self.line_queue)

        self.teaching = False
        self.running = False

        self._build_ui()
        self.after(50, self._poll_queue)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        ctl = ttk.Frame(root)
        ctl.pack(fill="x", pady=(0, 8))

        self.btn_teach = ttk.Button(ctl, text="Start Teach (Record)", command=self.on_teach_toggle)
        self.btn_teach.pack(side="left", padx=(0, 8))

        self.btn_run = ttk.Button(ctl, text="Run Trajectory", command=self.on_run_clicked)
        self.btn_run.pack(side="left", padx=(0, 8))

        self.btn_stop_all = ttk.Button(ctl, text="Stop All", command=self.stop_all)
        self.btn_stop_all.pack(side="left", padx=(0, 8))

        self.btn_open_dir = ttk.Button(ctl, text="Open CSV Dir…", command=self.open_csv_dir)
        self.btn_open_dir.pack(side="left", padx=(0, 8))

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(root, textvariable=self.status_var).pack(fill="x", pady=(0, 8))

        self.log = tk.Text(root, height=28)
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")

        note = ttk.Label(root, text=(
            "Make sure topics in your record/playback scripts match your system. "
            "Edit ROS_SETUP/ROBOT_IP in this file if needed."
        ))
        note.pack(fill="x", pady=(8, 0))

    def _append_log(self, line: str):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _poll_queue(self):
        try:
            while True:
                line = self.line_queue.get_nowait()
                self._append_log(line)
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    # Actions
    def on_teach_toggle(self):
        if not self.teaching:
            self.start_teach()
        else:
            self.stop_teach()

    def start_teach(self):
        self._append_log("Starting gravity compensation controller...")
        self.status_var.set("Launching gravity compensation...")
        self.teach_pg.start(bash_cmd(
            "ros2 launch franka_bringup example.launch.py "
            "controller_name:=gravity_compensation_example_controller"
        ))

        time.sleep(1.0)
        self._append_log("Starting joint recorder... (stop to save CSV)")
        self.teach_pg.start(bash_cmd("python3 record_joint_trajectory.py"))

        self.teaching = True
        self.btn_teach.configure(text="Stop Teach (Save)")
        self.status_var.set("Recording… Move arm by hand to teach.")

    def stop_teach(self):
        self._append_log("Stopping teach (sending SIGINT)…")
        self.teach_pg.terminate_all(sig=signal.SIGINT)
        time.sleep(0.5)
        self.teach_pg.terminate_all(sig=signal.SIGTERM)
        self.teach_pg.clear_finished()

        self.teaching = False
        self.btn_teach.configure(text="Start Teach (Record)")
        self.status_var.set("Teach stopped. Check CSV in current directory.")

    def on_run_clicked(self):
        csv_path = filedialog.askopenfilename(
            title="Select trajectory CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not csv_path:
            return
        self.start_run(csv_path)

    def start_run(self, csv_path: str):
        if self.running:
            messagebox.showwarning("Already running", "A playback is already running.")
            return

        self._append_log(f"Selected CSV: {csv_path}")
        self.status_var.set("Starting MoveIt and controllers…")
        self.run_pg.start(bash_cmd(
            f"ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:={ROBOT_IP}"
        ))

        def delayed_start():
            time.sleep(3.0)
            self.line_queue.put("Starting trajectory playback…")
            self.run_pg.start(bash_cmd(
                f"python3 playback_joint_trajectory.py '{csv_path}'"
            ))
        threading.Thread(target=delayed_start, daemon=True).start()

        self.running = True
        self.status_var.set("Running trajectory…")
        self.btn_run.configure(text="Running…")
        threading.Thread(target=self._watch_run_finish, daemon=True).start()

    def _watch_run_finish(self):
        while self.run_pg.is_alive():
            time.sleep(0.5)
        self.running = False
        # UI updates must be done in main thread; schedule via after
        self.after(0, lambda: self.btn_run.configure(text="Run Trajectory"))
        self.after(0, lambda: self.status_var.set("Run finished."))

    def stop_all(self):
        self._append_log("Stopping all processes…")
        self.teach_pg.terminate_all(sig=signal.SIGINT)
        self.run_pg.terminate_all(sig=signal.SIGINT)
        time.sleep(0.5)
        self.teach_pg.terminate_all(sig=signal.SIGTERM)
        self.run_pg.terminate_all(sig=signal.SIGTERM)
        self.teach_pg.clear_finished()
        self.run_pg.clear_finished()
        self.status_var.set("All processes signaled to stop.")

    def open_csv_dir(self):
        cwd = os.getcwd()
        self._append_log(f"Opening directory: {cwd}")
        if os.name == "posix":
            subprocess.Popen(["xdg-open", cwd])
        elif os.name == "nt":
            os.startfile(cwd)  # type: ignore
        else:
            messagebox.showinfo("Info", f"CSV directory: {cwd}")

def main():
    app = FR3TeachRunGUI()
    app.mainloop()

if __name__ == "__main__":
    main()
