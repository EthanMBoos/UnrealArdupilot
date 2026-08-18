#!/usr/bin/env python3
"""Run the documented 120/240/480 Hz ten-second convergence check."""

import argparse
import csv
import json
import pathlib
import subprocess
import tempfile


def run(command):
    return subprocess.run(command, check=True, text=True, capture_output=True)


def load_rows(path):
    with path.open(newline="") as stream:
        return {int(row["tick"]): row for row in csv.DictReader(stream)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uvd", default="build/uvd")
    parser.add_argument("--run", default="examples/runs/headless.json")
    args = parser.parse_args()
    source_run = pathlib.Path(args.run).resolve()
    base = json.loads(source_run.read_text())
    source_aircraft = (source_run.parent / base["aircraft"]["path"]).resolve()
    with tempfile.TemporaryDirectory(prefix="uvd-convergence-") as temporary:
        root = pathlib.Path(temporary)
        aircraft = root / "aircraft.json"
        aircraft.write_bytes(source_aircraft.read_bytes())
        base["aircraft"] = {"path":"aircraft.json"}
        base["clock"]["fixed_dt_s"] = 1/120
        base["stop"] = {"duration_s":10.0}
        base_run = root / "base.json"
        base_run.write_text(json.dumps(base,indent=2)+"\n")
        trim_dir = root / "trim"
        run([args.uvd,"trim",str(base_run),"--output",str(trim_dir)])
        operating = json.loads((trim_dir/"results/operating_point.json").read_text())
        outputs = {}
        for rate in (120,240,480):
            config = dict(base)
            config["run_id"] = f"convergence_{rate}hz"
            config["clock"] = {"fixed_dt_s": 1 / rate, "motion_solver": "rk4"}
            config["initial_state"] = operating["state"]
            command = operating["command"]
            pulse = dict(command); pulse["elevator"] += 0.01
            config["controls"] = {"input_boundary":"aircraft_command","mode":"absolute","final":"hold","schedule":[
                {"apply_tick":0,"values":command},
                {"apply_tick":rate,"time_s":1.0,"values":{"elevator":pulse["elevator"]}},
                {"apply_tick":int(1.1*rate),"time_s":1.1,"values":{"elevator":command["elevator"]}}]}
            config["stop"] = {"duration_s":10.0}
            path = root/f"run-{rate}.json";path.write_text(json.dumps(config,indent=2)+"\n")
            output = root/f"result-{rate}";run([args.uvd,"simulate",str(path),"--output",str(output)])
            outputs[rate] = load_rows(output/"signals.csv")
        columns = ["pn_m","pe_m","pd_m","qw","qx","qy","qz","u_mps","v_mps","w_mps","p_radps","q_radps","r_radps"]
        scales = [100,100,100,1,1,1,1,25,25,25,1,1,1]
        maximum = 0.0
        for tick, coarse in outputs[240].items():
            fine = outputs[480][tick*2]
            squared = sum(((float(coarse[name])-float(fine[name]))/scale)**2 for name,scale in zip(columns,scales))
            maximum = max(maximum,squared**0.5)
        report = {"passed":maximum<0.001,"rates_hz":[120,240,480],"comparison":"240 Hz vs 480 Hz at common times","max_scaled_norm":maximum,"limit":0.001}
        print(json.dumps(report,indent=2))
        if not report["passed"]:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
