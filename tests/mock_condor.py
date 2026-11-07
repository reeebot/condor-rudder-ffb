"""Fake Condor UDP sender: scripted flight (ground roll, climb-out, 8 deg slip,
coordinated, then paused). Usage: python mock_condor.py PORT
"""
# 0-4 s ground roll, 4-8 s climb-out, 8-12 s slip (yaw string -8 deg),
# 12-13 s coordinated, 13-15.5 s paused (sim clock frozen, packets still sent), then silence.
import socket, time, math, sys
port = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t0 = time.monotonic(); simtime = 12.0
while True:
    t = time.monotonic() - t0
    if t > 15.5: break
    if t < 4:    ias = 3 + 22 * t / 4; wh = 0.0; yaw = 0.01 * math.sin(t * 7); rough = 0.6
    elif t < 8:  ias = 25 + 3 * (t - 4); wh = 2 + 10 * (t - 4); yaw = 0.0; rough = 0.0
    elif t < 12: ias = 30; wh = 50; yaw = math.radians(-8); rough = 0.0
    else:        ias = 30; wh = 50; yaw = 0.0; rough = 0.0
    if t < 13: simtime += 0.02 / 3600
    msg = (f"time={simtime}\r\nairspeed={ias}\r\naltitude={500+wh}\r\nvario=0.5\r\n"
           f"yawstringangle={yaw}\r\nslipball=0.0\r\nvx={ias}\r\nvy=0.0\r\nvz=0.0\r\n"
           f"height={wh+0.8}\r\nwheelheight={wh}\r\nturbulencestrength=0.3\r\n"
           f"surfaceroughness={rough}\r\nhudmessages=a=b;c\r\n")
    s.sendto(msg.encode(), ("127.0.0.1", port))
    time.sleep(0.02)
