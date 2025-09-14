#!/usr/bin/env python3
import psutil, time, argparse, sys, os

p = argparse.ArgumentParser()
p.add_argument("pid", type=int, nargs='?', help="PID to monitor; default=self")
p.add_argument("--name", type=str, help="match process name/cmdline instead of pid")
p.add_argument("--interval", type=float, default=0.5)
a = p.parse_args()

def pick_pid_by_name(name: str):
    for proc in psutil.process_iter(attrs=["pid","name","cmdline"]):
        cmd = " ".join(proc.info.get("cmdline") or [])
        if name.lower() in (proc.info.get("name") or "").lower() or name.lower() in cmd.lower():
            return proc.info["pid"]
    return None

pid = a.pid
if a.name and not pid:
    pid = pick_pid_by_name(a.name)
if not pid:
    pid = os.getpid()  # 默认监控自己

try:
    proc = psutil.Process(pid)
except psutil.NoSuchProcess:
    sys.exit(f"No such process: {pid}")

peak = 0.0
print(f"Sampling PID {pid}... Ctrl+C to stop.")
proc.cpu_percent(interval=None)  # 预热
try:
    while True:
        val = proc.cpu_percent(interval=a.interval)
        peak = max(peak, val)
        print(f"now={val:6.1f}%  peak={peak:6.1f}%")
except KeyboardInterrupt:
    pass
print(f"Peak %CPU: {peak:.1f}%")
