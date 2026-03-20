#!/usr/bin/env python3
import csv
import os
import shlex
import signal
import subprocess
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog

# --------- CONFIG (edit if needed) ---------
ROS_SETUP = "source ~/franka_ws/install/setup.bash"
ROBOT_IP = "172.16.0.2"  # change if needed
TEACH_NAMESPACE = "NS_1"
GRIPPER_OPEN_WIDTH = 0.08
GRIPPER_CLOSE_WIDTH = 0.0
GRIPPER_SPEED = 0.1
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
        self.geometry("900x720")
        self.minsize(900, 720)

        self.line_queue = queue.Queue()
        self.teach_pg = ProcessGroup(self.line_queue)
        self.run_pg = ProcessGroup(self.line_queue)
        self.gripper_node_pg = ProcessGroup(self.line_queue)
        self.gripper_pg = ProcessGroup(self.line_queue)

        self.teaching = False
        self.running = False
        self.gravity_mode = False
        self.current_recording_filename = None
        self.teach_start_time_ns = None
        self.recorded_gripper_events = []

        self._build_ui()
        self._refresh_controls()
        self.after(50, self._poll_queue)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)

        controls = ttk.Frame(root)
        controls.pack(fill="x", pady=(0, 8))

        top_row = ttk.Frame(controls)
        top_row.pack(fill="x", pady=(0, 6))

        left_top = ttk.Frame(top_row)
        left_top.pack(side="left")

        self.btn_teach = ttk.Button(left_top, text="Start Teach (Record)", command=self.on_teach_toggle)
        self.btn_teach.pack(side="left", padx=(0, 8))

        self.btn_run = ttk.Button(left_top, text="Run Trajectory", command=self.on_run_clicked)
        self.btn_run.pack(side="left")

        self.btn_kill = tk.Button(
            top_row,
            text="Kill",
            command=self.kill_all,
            bg="#c62828",
            fg="white",
            activebackground="#b71c1c",
            activeforeground="white",
            relief="raised",
            padx=16,
        )
        self.btn_kill.pack(side="right")

        second_row = ttk.Frame(controls)
        second_row.pack(fill="x")

        self.btn_gravity = ttk.Button(second_row, text="Start Gravity Mode", command=self.on_gravity_toggle)
        self.btn_gravity.pack(side="left", padx=(0, 8))

        self.btn_gripper_open = ttk.Button(second_row, text="Open Gripper", command=self.open_gripper)
        self.btn_gripper_open.pack(side="left", padx=(0, 8))

        self.btn_gripper_close = ttk.Button(second_row, text="Close Gripper", command=self.close_gripper)
        self.btn_gripper_close.pack(side="left")

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(root, textvariable=self.status_var).pack(fill="x", pady=(0, 8))

        self.log = tk.Text(root, height=22)
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")

        footer = ttk.Frame(root)
        footer.pack(fill="x", pady=(8, 0))

        note = ttk.Label(footer, text=(
            "Make sure topics in your record/playback scripts match your system. "
            "Edit ROS_SETUP/ROBOT_IP in this file if needed."
        ))
        note.pack(side="left", fill="x", expand=True)

        self.btn_open_dir = ttk.Button(footer, text="Open CSV Dir…", command=self.open_csv_dir)
        self.btn_open_dir.pack(side="right")

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

    def _refresh_controls(self):
        if self.teaching:
            self.btn_teach.configure(state="normal")
            self.btn_run.configure(state="disabled")
            self.btn_gravity.configure(state="disabled")
            self.btn_open_dir.configure(state="disabled")
        else:
            run_enabled = "disabled" if self.running else "normal"
            gravity_enabled = "disabled" if self.running else "normal"
            teach_enabled = "disabled" if self.running else "normal"
            open_dir_enabled = "disabled" if self.running else "normal"
            self.btn_teach.configure(state=teach_enabled)
            self.btn_run.configure(state=run_enabled)
            self.btn_gravity.configure(state=gravity_enabled)
            self.btn_open_dir.configure(state=open_dir_enabled)

        self.btn_gripper_open.configure(state="normal")
        self.btn_gripper_close.configure(state="normal")

        any_active = (
            self.teaching
            or self.running
            or self.gravity_mode
            or self.teach_pg.is_alive()
            or self.run_pg.is_alive()
            or self.gripper_node_pg.is_alive()
        )
        self.btn_kill.configure(state="normal" if any_active else "disabled")

    # Actions
    def on_gravity_toggle(self):
        if not self.gravity_mode:
            self.start_gravity_mode()
        else:
            self.stop_gravity_mode()

    def on_teach_toggle(self):
        if not self.teaching:
            self.start_teach()
        else:
            self.stop_teach()

    def launch_gravity_controller(self):
        self._append_log("Starting gravity compensation controller...")
        self.status_var.set("Launching gravity compensation...")
        self.teach_pg.start(bash_cmd(
            "ros2 launch franka_bringup example.launch.py "
            "controller_name:=gravity_compensation_example_controller"
        ))
        self.ensure_gripper_node_running()
        self.gravity_mode = True
        self.btn_gravity.configure(text="Stop Gravity Mode")
        self._refresh_controls()

    def start_gravity_mode(self):
        if self.teach_pg.is_alive():
            messagebox.showwarning(
                "Teach processes running",
                "Teach or gravity mode processes are already running. Stop them first if needed."
            )
            return
        self.launch_gravity_controller()
        self.status_var.set("Gravity compensation active.")
        self._refresh_controls()

    def stop_gravity_mode(self):
        self._append_log("Stopping gravity compensation...")
        self.teach_pg.terminate_all(sig=signal.SIGINT)
        time.sleep(0.5)
        self.teach_pg.terminate_all(sig=signal.SIGTERM)
        self.teach_pg.clear_finished()
        self.gravity_mode = False
        self.teaching = False
        self.btn_gravity.configure(text="Start Gravity Mode")
        self.btn_teach.configure(text="Start Teach (Record)")
        self.status_var.set("Gravity compensation stopped.")
        self._refresh_controls()

    def start_teach(self):
        custom_name = simpledialog.askstring(
            "Recording File Name",
            "Optional: enter a CSV filename for this recording. Leave blank to use the default timestamped name.",
            parent=self,
        )
        if custom_name is None:
            custom_name = ""
        custom_name = custom_name.strip()

        if not self.gravity_mode:
            self.launch_gravity_controller()
            time.sleep(1.0)
        else:
            self._append_log("Gravity compensation already active; starting recorder only...")

        if custom_name:
            if not custom_name.endswith(".csv"):
                custom_name += ".csv"
            recording_filename = custom_name
            recorder_cmd = f"python3 record_joint_trajectory.py {shlex.quote(custom_name)}"
            self._append_log(f"Starting joint recorder... output file: {custom_name}")
        else:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            recording_filename = f"joint_trajectory_{timestamp}.csv"
            recorder_cmd = f"python3 record_joint_trajectory.py {shlex.quote(recording_filename)}"
            self._append_log(f"Starting joint recorder... output file: {recording_filename}")

        self.current_recording_filename = recording_filename
        self.teach_start_time_ns = time.time_ns()
        self.recorded_gripper_events = []
        self.teach_pg.start(bash_cmd(recorder_cmd))

        self.teaching = True
        self.btn_teach.configure(text="Stop Teach (Save)")
        self.status_var.set("Recording… Move arm by hand to teach.")
        self._refresh_controls()

    def stop_teach(self):
        self._append_log("Stopping teach (sending SIGINT)…")
        self.teach_pg.terminate_all(sig=signal.SIGINT)
        time.sleep(0.5)
        self.teach_pg.terminate_all(sig=signal.SIGTERM)
        self.teach_pg.clear_finished()

        self.append_gripper_events_to_csv()
        self.teaching = False
        self.gravity_mode = False
        self.teach_start_time_ns = None
        self.current_recording_filename = None
        self.recorded_gripper_events = []
        self.btn_teach.configure(text="Start Teach (Record)")
        self.btn_gravity.configure(text="Start Gravity Mode")
        self.status_var.set("Teach stopped. Check CSV in current directory.")
        self._refresh_controls()


    def record_gripper_event(self, event_name: str):
        if not self.teaching or self.teach_start_time_ns is None:
            return
        timestamp_ns = max(0, time.time_ns() - self.teach_start_time_ns)
        self.recorded_gripper_events.append((timestamp_ns, event_name))

    def append_gripper_events_to_csv(self):
        if not self.current_recording_filename or not self.recorded_gripper_events:
            return
        if not os.path.exists(self.current_recording_filename):
            self._append_log(
                f"Recording file not found for gripper event append: {self.current_recording_filename}"
            )
            return
        with open(self.current_recording_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            for timestamp_ns, event_name in self.recorded_gripper_events:
                writer.writerow([timestamp_ns, 'gripper', event_name, '', '', '', '', '', '', ''])
        self._append_log(
            f"Appended {len(self.recorded_gripper_events)} gripper event(s) to {self.current_recording_filename}"
        )

    def launch_gripper_node(self):
        self._append_log("Starting gripper node...")
        self.gripper_node_pg.start(bash_cmd(
            f"ros2 launch franka_gripper gripper.launch.py robot_ip:={ROBOT_IP} namespace:={TEACH_NAMESPACE} arm_id:=fr3"
        ))

    def ensure_gripper_node_running(self):
        if not self.gripper_node_pg.is_alive():
            self.launch_gripper_node()
            time.sleep(2.0)

    def get_gripper_action_candidates(self):
        return [
            "/franka_gripper/move",
            f"/{TEACH_NAMESPACE}/franka_gripper/move",
            "/fr3_gripper/move",
            f"/{TEACH_NAMESPACE}/fr3_gripper/move",
        ]

    def send_gripper_command(self, width: float, label: str):
        self._append_log(f"Sending gripper {label.lower()} command...")

        def worker():
            self.ensure_gripper_node_running()

            action_list_proc = subprocess.run(
                bash_cmd("ros2 action list"),
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            action_names = {
                line.strip() for line in action_list_proc.stdout.splitlines() if line.strip()
            }

            action_name = next(
                (candidate for candidate in self.get_gripper_action_candidates() if candidate in action_names),
                None,
            )
            if action_name is None:
                self.line_queue.put(
                    "No gripper move action server found. Expected one of: "
                    + ", ".join(self.get_gripper_action_candidates())
                )
                return

            self.line_queue.put(f"Sending {label} command to {action_name}")
            goal = shlex.quote(f"{{width: {width}, speed: {GRIPPER_SPEED}}}")
            self.gripper_pg.start(
                bash_cmd(
                    f"ros2 action send_goal {action_name} franka_msgs/action/Move {goal}"
                )
            )

        threading.Thread(target=worker, daemon=True).start()

    def open_gripper(self):
        self.record_gripper_event("open")
        self.send_gripper_command(GRIPPER_OPEN_WIDTH, "Open")

    def close_gripper(self):
        self.record_gripper_event("close")
        self.send_gripper_command(GRIPPER_CLOSE_WIDTH, "Close")

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
        self._refresh_controls()
        threading.Thread(target=self._watch_run_finish, daemon=True).start()

    def _watch_run_finish(self):
        while self.run_pg.is_alive():
            time.sleep(0.5)
        self.running = False
        # UI updates must be done in main thread; schedule via after
        self.after(0, lambda: self.btn_run.configure(text="Run Trajectory"))
        self.after(0, lambda: self.status_var.set("Run finished."))
        self.after(0, self._refresh_controls)

    def kill_all(self):
        self._append_log("Killing active processes…")
        self.teach_pg.terminate_all(sig=signal.SIGTERM)
        self.run_pg.terminate_all(sig=signal.SIGTERM)
        self.gripper_node_pg.terminate_all(sig=signal.SIGTERM)
        self.gripper_pg.terminate_all(sig=signal.SIGTERM)
        time.sleep(0.3)
        self.teach_pg.terminate_all(sig=signal.SIGKILL)
        self.run_pg.terminate_all(sig=signal.SIGKILL)
        self.gripper_node_pg.terminate_all(sig=signal.SIGKILL)
        self.gripper_pg.terminate_all(sig=signal.SIGKILL)
        self.teach_pg.clear_finished()
        self.run_pg.clear_finished()
        self.gripper_node_pg.clear_finished()
        self.gripper_pg.clear_finished()
        self.teaching = False
        self.gravity_mode = False
        self.running = False
        self.teach_start_time_ns = None
        self.current_recording_filename = None
        self.recorded_gripper_events = []
        self.btn_gravity.configure(text="Start Gravity Mode")
        self.btn_teach.configure(text="Start Teach (Record)")
        self.btn_run.configure(text="Run Trajectory")
        self.status_var.set("Active processes killed.")
        self._refresh_controls()

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
