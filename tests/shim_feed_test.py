"""Stick-shim feed: ShimIn parses the shim's STATE heartbeat and the model turns Condor's own
stall / VNE shake into pedal flutter (and falls back to the IAS model without it)."""
import math, os, socket, sys, time
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
check(shim.shake() == (1500, 1600, False) and shim.spring_k == 4997, f"STATE parsed, SETP ignored: {shim.shake()}")
tx.sendto(b"1794016606093 STATE seq=2 stall1=0 stall2=3500 spring_k=10000 broken=1", ("127.0.0.1", PORT))
time.sleep(0.05); shim.poll()
check(shim.shake() == (0, 3500, True), f"broken flag parsed: {shim.shake()}")
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

# --- wing break: sudden drop to a residual that fades with airspeed
mod = m.ForceModel(dict(cfg))
st = dict(air, airspeed=136 * 0.514444)
for i in range(60):                                   # VNE flutter at full for 2 s
    st["time"] = 1.0 + i * 0.033
    t = mod.update(st, 1 / 30, 0.0, (0, 10000, False))
before = t.slow_f
st["time"] += 0.033
t = mod.update(st, 1 / 30, 0.0, (0, 0, True))        # Condor cuts it: shim says broken
r = cfg["failure_residual"]
check(mod.broken and abs(t.slow_f - before * r) <= before * r * 0.04, f"break: flutter {before} -> {t.slow_f} (residual {r})")
st["airspeed"] = 68 * 0.514444
for i in range(60):
    st["time"] += 0.033
    t = mod.update(st, 1 / 30, 0.0, (0, 0, False))
check(abs(t.slow_f - before * r * 0.5) <= 3, f"half the airspeed -> half the residual ({t.slow_f})")
t = mod.neutral(1 / 30)
check(not mod.broken, "new flight / pause clears the break")

mod = m.ForceModel(dict(cfg, failure_residual=0.0))
st = dict(air, airspeed=136 * 0.514444)
for i in range(30):
    st["time"] = 1.0 + i * 0.033
    mod.update(st, 1 / 30, 0.0, (0, 10000, False))
t = mod.update(st, 1 / 30, 0.0, (0, 0, True))
check(t.slow_f == 0 and t.fast_f == 0, "failure_residual = 0 -> everything stops (Condor's way)")

# --- linkage: damping grows with airspeed; limp load setting
t, i = run(None, kt=0.0)
check(abs(t.damp - cfg["damping_rest"]) < 1e-9 and t.move_back == 0, f"stopped: limp, damping {t.damp}")
t, i = run(None, kt=100.0)
check(abs(t.damp - (cfg["damping_rest"] + cfg["damping_per_100kt"])) < 0.002, f"100 kt: damping {t.damp:.3f}")
t, i = run(None, kt=0.0, limp_load_pct=35.0)
check(t.sf == 35, f"limp_load_pct 35 -> scale {t.sf}")

# --- stick buffet (sent to the shim): airbrakes and high speed
def run_ab(kt, ab, n=40, **kw):
    mod = m.ForceModel(dict(cfg, **kw))
    st = dict(air, airspeed=kt * 0.514444)
    for i in range(n):
        st["time"] = 1.0 + i * 0.033
        mod.update(st, 1 / 30, ab, (0, 0, False))
    return mod.info.get("stick_buffet", 0.0)
v = run_ab(cfg["airbrake_vib_full_kt"], 1.0)
check(abs(v - cfg["stick_airbrake_buffet"] / 100) < 0.01, f"full brakes at full speed -> stick buffet {v:.2f}")
v = run_ab(60, 1.0)
check(0.1 < v < cfg["stick_airbrake_buffet"] / 100, f"full brakes at 60 kt -> {v:.2f} (already strong)")
v = run_ab(60, 0.0)
check(v == 0, f"brakes closed at 60 kt -> {v}")
v = run_ab(cfg["hs_buffet_full_kt"], 0.0)
check(abs(v - cfg["stick_hs_buffet"] / 100) < 0.01, f"high-speed buffet full -> stick {v:.2f}")
v = run_ab(cfg["airbrake_vib_full_kt"], 1.0, stick_airbrake_buffet=0)
check(v == 0, "stick_airbrake_buffet = 0 -> off")

# --- stick gusts: band-passed load factor / roll rate -> forces, not stiffness
mod = m.ForceModel(dict(cfg))
st = dict(air, airspeed=60 * 0.514444, gforce=1.0, rollrate=0.0)
for i in range(90):
    st["time"] = 1.0 + i * 0.033; mod.update(st, 1 / 30, 0.0, (0, 0, False))
check(abs(mod.info["gust_pitch"]) < 1e-3, f"steady 1 g -> no gust force ({mod.info['gust_pitch']:.4f})")
st["gforce"] = 1.5
peak = 0
for i in range(6):
    st["time"] += 0.033; mod.update(st, 1 / 30, 0.0, (0, 0, False)); peak = max(peak, mod.info["gust_pitch"])
check(0.04 < peak <= cfg["stick_gust_max"] / 100, f"+0.5 g bump -> push forward {peak * 100:.1f}% (cap {cfg['stick_gust_max']}%)")
for i in range(150):
    st["time"] += 0.033; mod.update(st, 1 / 30, 0.0, (0, 0, False))
check(abs(mod.info["gust_pitch"]) < 0.005, f"held 1.5 g (steady turn) -> fades out ({mod.info['gust_pitch'] * 100:.2f}%)")
st["rollrate"] = math.radians(15)
peak = 0
for i in range(6):
    st["time"] += 0.033; mod.update(st, 1 / 30, 0.0, (0, 0, False)); peak = min(peak, mod.info["gust_roll"])
check(peak < -0.02, f"roll-rate kick +15 deg/s -> stick pushed against it ({peak * 100:.1f}%)")
st["wheelheight"] = 0.0
mod.update(st, 1 / 30, 0.0, (0, 0, False))
check(mod.info["gust_pitch"] == 0 and mod.info["gust_roll"] == 0, "on the ground -> no gust forces")

# --- end to end: bridge (dry run) sends BUFFET to the shim's command port while the shim is live
import threading
CPORT, SPORT, KPORT = 55395, 55396, 55397
bcfg = dict(cfg, condor_listen_port=CPORT, shim_udp_port=SPORT, shim_cmd_port=KPORT, forward_to="")
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.bind(("127.0.0.1", KPORT)); rx.settimeout(0.2)
th = threading.Thread(target=m.run_bridge, args=(bcfg, True, None, 2.5), daemon=True)
th.start()
t0 = time.monotonic(); simtime = 12.0; got = []
while time.monotonic() - t0 < 2.4:
    simtime += 0.02 / 3600
    gf = 1.0 + 0.4 * math.sin((time.monotonic() - t0) * 6)
    msg = (f"time={simtime}\r\nairspeed={140 * 0.514444}\r\naltitude=900\r\nyawstringangle=0\r\n"
           f"vx=70\r\nvy=0\r\nvz=0\r\nheight=400\r\nwheelheight=400\r\ngforce={gf}\r\nrollrate=0.0\r\n")
    tx.sendto(msg.encode(), ("127.0.0.1", CPORT))
    tx.sendto(b"0 STATE seq=1 stall1=0 stall2=0 spring_k=10000 broken=0", ("127.0.0.1", SPORT))
    try:
        while True:
            d, _ = rx.recvfrom(256); got.append(d.decode())
    except socket.timeout:
        pass
    except BlockingIOError:
        pass
    time.sleep(0.02)
th.join(3)
gains = [int(g.split("gain=")[1].split()[0]) for g in got if g.startswith("BUFFET gain=")]
pitches = [int(g.split("pitch=")[1].split()[0]) for g in got if "pitch=" in g]
check(pitches and max(abs(x) for x in pitches) > 300, f"bridge -> shim gust pitch forces, peak {max(abs(x) for x in pitches or [0])}")
check(len(gains) > 10 and max(gains) > 1000 and gains[-1] == 0,
      f"bridge -> shim: {len(gains)} BUFFET commands, peak gain {max(gains or [0])} at 140 kt, 0 once Condor stops")
print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
