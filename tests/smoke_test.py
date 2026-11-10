"""End-to-end smoke test: fake Condor -> bridge -> fake CLS2Sim. No hardware needed.

    python tests/smoke_test.py

Close run.bat first (same UDP ports 55278 / 15090). Takes ~20 s.
"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def main():
    tmp = tempfile.mkdtemp()
    log = os.path.join(tmp, "cls2sim.jsonl")
    ini = os.path.join(tmp, "settings.ini")
    cls = subprocess.Popen([sys.executable, os.path.join(HERE, "mock_cls2sim.py"), "15090", log])
    time.sleep(0.5)
    condor = subprocess.Popen([sys.executable, os.path.join(HERE, "mock_condor.py"), "55278"])
    try:
        run = subprocess.run([sys.executable, os.path.join(ROOT, "condor_brunner_ffb.py"),
                              "--settings", ini, "--run-seconds", "14"],
                             capture_output=True, text=True, timeout=90)
    finally:
        condor.kill()
        time.sleep(0.3)
        cls.kill()
    out = run.stdout.replace("\r", "\n")
    recs = [json.loads(ln) for ln in open(log)]
    sets = [r for r in recs if r["cmd"] == "0xce"]
    move_back = [r["val"] for r in sets if r.get("sid") == "0x40"]
    trims = [r["val"] for r in sets if r.get("sid") == "0x90"]
    overrides = [r for r in recs if r["cmd"] == "0xd1"]

    checks = [
        ("bridge exited cleanly", run.returncode == 0 and "Traceback" not in run.stderr),
        ("no malformed / out-of-range packets", not [r for r in recs if r.get("bad") or r["status"]]),
        ("saw ground roll and airborne phases", "GROUND" in out and "AIR " in out),
        ("limp at low speed, airflow centring later", 0 in move_back and 1 in move_back),
        ("yaw feel: rudder floats downwind in the left slip (trim < -0.1)", any(t < -0.1 for t in trims)),
        ("overrides released at the end", bool(overrides) and overrides[-1]["axis"] == 0),
        ("pedals handed back to CLS2Sim", "Overrides released" in out),
    ]
    width = max(len(n) for n, _ in checks)
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name:{width}s}")
    failed = [n for n, ok in checks if not ok]
    print(f"\n{len(recs)} CLS2Sim requests checked. " + ("ALL PASS" if not failed else f"{len(failed)} FAILED"))
    if failed:
        print("\n--- bridge stdout (tail) ---\n" + "\n".join(out.splitlines()[-15:]))
        print("--- stderr ---\n" + run.stderr[-2000:])
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
