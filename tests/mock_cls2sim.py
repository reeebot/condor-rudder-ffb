"""Fake CLS2Sim remote interface (protocol v2) for testing without hardware.
Decodes every request, checks packet layouts and value ranges, logs JSON lines.
Usage: python mock_cls2sim.py PORT LOGFILE [mid]   (add "mid" for Message Identifier mode)
"""
import socket, struct, sys, json, time
port = int(sys.argv[1]); log = open(sys.argv[2], "a"); MID = len(sys.argv) > 3
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", port))
SET_FMT = {0x40: "B", 0x20: "H", 0x90: "f", 0xC0: "B", 0xC1: "I", 0xC2: "I", 0xC3: "I", 0xC4: "I"}
state = {"trim": 0.0, "pos": 0.0}
def reply(body=b"", status=0):
    pkt = struct.pack("<B", status) + body
    return struct.pack("<H", len(pkt)) + pkt
while True:
    d, a = s.recvfrom(8192)
    mid = b""
    if MID: mid, d = d[:4], d[4:]
    cmd = struct.unpack_from("<I", d, 0)[0]
    rec = {"t": time.time(), "cmd": hex(cmd), "len": len(d)}
    st = 0; body = b""
    if cmd == 0xCE:
        axis, sid = struct.unpack_from("<II", d, 4)
        f = SET_FMT.get(sid)
        exp = 12 + struct.calcsize("<" + f) if f else None
        if f is None or len(d) != exp:
            st = 4; rec["bad"] = f"sid {sid:#x} len {len(d)} expected {exp}"
        else:
            v = struct.unpack_from("<" + f, d, 12)[0]
            rec.update(axis=axis, sid=hex(sid), val=v)
            if sid == 0x90: state["trim"] = v
            if sid == 0x20 and not (20 <= v <= 999): rec["bad"] = "scale out of range"
            if sid == 0x90 and not (-1 <= v <= 1): rec["bad"] = "trim out of range"
            if sid == 0xC3 and v < 10: rec["bad"] = "slow interval < 10"
    elif cmd == 0xD1:
        axis, oid = struct.unpack_from("<II", d, 4); rec.update(axis=axis, oid=oid)
        if len(d) != 12: st = 4; rec["bad"] = "override len"
    elif cmd == 0xD0:
        axis, did = struct.unpack_from("<II", d, 4); rec.update(axis=axis, did=hex(did))
        state["pos"] += (state["trim"] - state["pos"]) * 0.3
        body = struct.pack("<Hf", 5, state["pos"]) + struct.pack("<Hf", 6, 0.0)
    elif cmd == 0xCD:
        body = struct.pack("<HHH", 5, 1, 31) + struct.pack("<HHH", 6, 0, 0)
    elif cmd == 0xD3:
        body = struct.pack("<BBH", 3, 0, 0)
    elif cmd == 0xD8:
        rec["wd"] = struct.unpack_from("<IH", d, 4)
        if len(d) != 10: st = 4; rec["bad"] = "watchdog len"
    else:
        st = 2; rec["bad"] = "unknown cmd"
    rec["status"] = st
    log.write(json.dumps(rec) + "\n"); log.flush()
    r = reply(body, st); s.sendto(r[:2] + mid + r[2:], a)
