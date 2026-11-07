"""Stick-shim feed: ShimIn parses the shim's STATE heartbeat and the model turns Condor's own
stall / VNE shake into pedal flutter (and falls back to the IAS model without it)."""
import os, socket, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import condor_brunner_ffb as m

fails = 0
def check(ok, msg):
    global fails
    print(("  PASS  " if ok else "  FAIL  ") + msg)
    fails += 0 if ok else 1

cfg = dict(m.DEFAULTS)
PORT = 55391
shim = m.ShimIn(PORT)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
check(shim.shake() is None, "no feed yet -> None")
tx.sendto(b"1794016606043 STATE seq=1 stall1=1500 stall2=1600 spring_k=4997", ("127.0.0.1", PORT))
tx.sendto(b"1794016606043 SETP fx=7 Stall1 triangle flags=0x80000004 gain=9999", ("127.0.0.1", PORT))
time.sleep(0.05); shim.poll()
check(shim.shake() == (1500, 1600) and shim.spring_k == 4997, f"STATE parsed, SETP ignored: {shim.shake()}")
check(shim.shake(now=time.monotonic() + 2) is None, "stale after 1 s -> None")

air = {"airspeed": 40 * 0.514444, "gforce": 1.0, "wheelheight": 50.0, "yawstringangle": 0.0,
       "vx": 20.0, "vy": 0.0, "vz": 0.0, "time": 1.0, "altitude": 800.0}
def run(shake, kt=40.0, n=40, **kw):
    mod = m.ForceModel(dict(cfg, **kw))
    st = dict(air, airspeed=kt * 0.514444)
    for i in range(n):
        st["time"] = 1.0 + i * 0.033
        t = mod.update(st, 1 / 30, 0.0, shake)
    return t, mod.info

t, i = run((0, 0))
check(t.slow_f == 0 and i["src"] == "condor", f"shim live, Condor not shaking at 40 kt -> no flutter ({t.slow_f})")
t, i = run((1250, 1250))
check(abs(t.slow_f - cfg["buffet_force_max"] * 0.5) <= 1, f"Condor stall gain 1250 -> half flutter ({t.slow_f})")
t, i = run((2600, 2600))
check(t.slow_f == cfg["buffet_force_max"] and t.fast_f == cfg["buffet_buzz_force_max"],
      f"Condor stall peak -> full flutter + deep-stall buzz ({t.slow_f}, {t.fast_f})")
t, i = run((0, 10000), kt=136)
check(t.slow_f == cfg["condor_vne_force_max"] and i["vne"] > 0, f"Stall2 alone at 136 kt -> VNE flutter ({t.slow_f})")
t, i = run(None, kt=30)
check(t.slow_f > 0 and i["src"] == "ias", f"no shim -> IAS model still flutters at 30 kt ({t.slow_f})")
t, i = run((0, 0), kt=30)
check(t.slow_f == 0, f"shim says no stall at 30 kt -> Condor wins over the IAS guess ({t.slow_f})")
t, i = run((2000, 2000), kt=30, stall_source="ias")
check(i["src"] == "ias", "stall_source=ias ignores the shim")
t, i = run(None, kt=30, stall_source="condor")
check(t.slow_f == 0 and i["src"] == "none", "stall_source=condor without shim -> no stall flutter")
print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
