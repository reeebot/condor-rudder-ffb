"""Fake Condor + pilot following airbrake_test beeps (short timings: lead/open/closed 3 s).
Usage: python mock_condor_ab.py yes|no   (whether Condor sends an "airbrakes" field)
"""
import socket, time, random, sys
send_field = sys.argv[1] == "yes"
LEAD, OPEN, CLOSED, CYC = 3.0, 3.0, 3.0, 3
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t0 = time.monotonic(); simt = 10.0; alt = 1500.0
def brakes(t):
    t -= 0.6                                   # pilot reaction
    if t < LEAD: return 0.0
    ph = (t - LEAD) % (OPEN + CLOSED)
    if (t - LEAD) >= CYC * (OPEN + CLOSED): return 0.0
    if ph < OPEN: return min(1.0, ph / 0.5)    # lever travel 0.5 s
    return max(0.0, 1.0 - (ph - OPEN) / 0.5)
while time.monotonic() - t0 < LEAD + CYC * (OPEN + CLOSED) + 2:
    t = time.monotonic() - t0; ab = brakes(t)
    vario = -0.8 - 2.5 * ab + random.gauss(0, 0.5)
    alt += vario * 0.02; simt += 0.02 / 3600
    msg = (f"time={simt}\r\nairspeed={25 + random.gauss(0, 0.4) - 1.5 * ab}\r\naltitude={alt}\r\n"
           f"vario={vario}\r\nnettovario={0.3 * vario + random.gauss(0, 0.4)}\r\n"
           f"yawstringangle={random.gauss(0, 0.01)}\r\nwheelheight=300\r\ngforce={1 + random.gauss(0, 0.05)}\r\n"
           f"flaps=2\r\nhudmessages=a;b\r\n")
    if send_field: msg += f"airbrakes={ab}\r\n"
    s.sendto(msg.encode(), ("127.0.0.1", 55278)); time.sleep(0.02)
