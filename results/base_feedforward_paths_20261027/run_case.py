"""One isolated active-path or unmodified passive trial, with full provenance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import zlib

p = argparse.ArgumentParser()
p.add_argument("tag")
p.add_argument("--mode", choices=["passive", "active", "zero_feedforward"], required=True)
p.add_argument("--viewer", action="store_true")
p.add_argument("--task", choices=["S11", "S12", "sinusoid", "S22"], required=True)
p.add_argument("--config", default=None)
args = p.parse_args()
here = Path(__file__).resolve().parent
root = here.parents[1]
args.config = args.config or str(here / f"{args.task}.yaml")
path_file = here / f"robot_path_{args.task}.yaml"
out = here / args.tag
out.mkdir(exist_ok=False)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID=str(90+zlib.crc32(args.tag.encode()) % 100),
           ROS_LOG_DIR=str(out / "ros"), PR2_DEBUG_TRACE="1",
           PR2_SIM_DEBUG_LOG=str(out / "bridge_debug.jsonl"),
           PR2_DEBUG_LOG=str(out / "qp_debug.jsonl"),
           BASE_PATH_MODE=args.mode, BASE_PATH_GAIN="0" if args.mode == "zero_feedforward" else "1",
           BASE_PATH_OUT=str(out), BASE_PATH_TRACE=str(out / "allocation.jsonl"), BASE_PATH_FILE=str(path_file))
sources = [here / f for f in ["angular_priority.py", "experimental_qp.py", "experimental_guard.py", "known_path.launch.py", "planned_base_path.py", "run_case.py"]]
sources.append(path_file)
before = json.loads((here / "production_hashes_before.json").read_text())
sources += [root / f for f in before]
sources += [Path(args.config)]
(out / "source_hashes.json").write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},indent=2))
(out / "control_condition.json").write_text(json.dumps(dict(mode=args.mode, task=args.task, robot_path_file=str(path_file),
    note="Independent configured base path. Original human and friction scene. "
         "Active: nominal planar transport + linear admittance correction, angular admittance priority. "
         "Passive: original QP. Zero-feedforward: new allocation with zero path velocity. "
         "Base filtering and 10Hz coordinator unchanged."),indent=2))


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL)
            proc.wait()


with (out / "logger.log").open("w") as lf, (out / "launch.log").open("w") as sf:
    logger = subprocess.Popen(["python3",str(root / "results/friction_paths_20261026/logger.py"),str(out / "raw.npz")],
                              env=env,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True)
    time.sleep(.8)
    cmd = ["ros2","launch",str(here / "known_path.launch.py"),
           "condition:=human_robot","robot_mode:=admittance",f"use_viewer:={str(args.viewer).lower()}",
           f"config_file:={args.config}",f"experiment_id:={args.tag}",f"output_root:={out}"]
    (out / "launch_command.json").write_text(json.dumps(cmd,indent=2))
    sim = subprocess.Popen(cmd,env=env,stdout=sf,stderr=subprocess.STDOUT,start_new_session=True)
    timeout = False
    try:
        sim.wait(timeout=150)
    except subprocess.TimeoutExpired:
        timeout = True
    finally:
        stop(sim)
        try:
            logger.wait(timeout=5)
        except subprocess.TimeoutExpired:
            stop(logger)
    (out / "exit.json").write_text(json.dumps(dict(launch_exit=sim.returncode,logger_exit=logger.returncode,timed_out=timeout),indent=2))
print(out,flush=True)
if timeout:
    raise SystemExit(1)
