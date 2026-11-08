/*
 * dinput8.dll shim for Condor 3 - log and boost the force feedback Condor sends to the stick.
 *
 * Drop dinput8.dll + ffb_shim.ini next to Condor.exe. Condor loads this file instead of the
 * Windows one; we load the real DirectInput from the system folder and pass every call
 * straight through. Only the force-feedback calls are touched:
 *
 *   - effect strengths are multiplied by the percentages in ffb_shim.ini (live reload):
 *       spring: stiffness, on a curve that boosts the low end and tapers into the stick's
 *               maximum at Condor's own top stiffness (~97 kt), so the force never maxes out
 *               part way through the travel (optional "assist" spring for more than that)
 *       shake:  Condor drives its stall/VNE shake with the effect gain, so that is what we
 *               scale (spilling into the magnitude once the gain is maxed)
 *   - linkage: friction, inertia and damping effects of our own on the stick, so moving it
 *     always means moving cables and a control surface with mass (more damping with speed)
 *   - wing break: Condor cuts its VNE flutter dead; we keep a share of it going (the other
 *     wing and the tail still flutter), fading as the airspeed drops
 *   - bridge buffet: the pedal bridge (which reads the airbrake lever and Condor's flight data)
 *     sends "BUFFET gain=.. period_ms=.. pitch=.. roll=.." to UDP 55292; we play the buffet as an
 *     extra shake and pitch/roll as gust forces (constant forces) on the stick
 *   - every effect Condor creates, changes, starts or stops is written to ffb_shim.log and
 *     sent as one text line over UDP to 127.0.0.1 (so the pedal bridge can follow Condor's
 *     own stall/VNE shake)
 *
 * Stick axes and buttons never pass through here, so there is no input lag.
 * Uninstall: delete dinput8.dll from the Condor folder.
 *
 * Build (32-bit, Condor.exe is x86):  see build.sh
 */
#define WIN32_LEAN_AND_MEAN
#define DIRECTINPUT_VERSION 0x0800
#include <winsock2.h>
#include <windows.h>
#include <dinput.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define SHIM_VERSION "0.5"

/* ------------------------------------------------------------------ vtable slots */
enum { DI8_CreateDevice = 3 };
enum {
    DEV_SetProperty = 6, DEV_GetDeviceInfo = 15, DEV_CreateEffect = 18,
    DEV_SendForceFeedbackCommand = 22, DEV_EnumEffectsInFile = 27
};
enum { FX_Release = 2, FX_GetParameters = 5, FX_SetParameters = 6, FX_Start = 7, FX_Stop = 8 };

/* ------------------------------------------------------------------ globals */
typedef HRESULT (WINAPI *PFN_DI8Create)(HINSTANCE, DWORD, REFIID, LPVOID *, LPUNKNOWN);

static HMODULE g_self, g_real;
static PFN_DI8Create p_DI8Create;
static CRITICAL_SECTION g_cs;
static char g_dir[MAX_PATH];
static int g_wide;

typedef struct {
    int spring, damper, shake, constant, master, spring_assist;
    int log, log_setp, max_mb, udp_port;
    double shake_curve;
    int break_residual;
    int link_friction, link_inertia, link_damper, link_damper_speed;
    int cmd_port, bridge_buffet, bridge_gusts;
} Cfg;
static Cfg g_cfg = {100, 100, 100, 100, 100, 0, 1, 1, 50, 55291, 1.0, 0, 0, 0, 0, 0, 55292, 100, 100};
static unsigned g_setp_calls;
static FILETIME g_ini_mtime;
static DWORD g_ini_check;

static FILE *g_log;
static long long g_log_bytes;
static DWORD g_last_flush;
static int g_log_capped;

static SOCKET g_sock = INVALID_SOCKET;
static int g_wsa_tried;

/* ------------------------------------------------------------------ vtable patching */
typedef struct { void **vtbl; int slot; void *orig; } Patch;
static Patch g_patch[64];
static int g_npatch;

static void *patch_slot(void **vtbl, int slot, void *hook)
{
    int i;
    DWORD old;
    for (i = 0; i < g_npatch; i++)
        if (g_patch[i].vtbl == vtbl && g_patch[i].slot == slot) return g_patch[i].orig;
    if (g_npatch >= (int)(sizeof g_patch / sizeof g_patch[0])) return NULL;
    if (vtbl[slot] == hook) return NULL;
    g_patch[g_npatch].vtbl = vtbl;
    g_patch[g_npatch].slot = slot;
    g_patch[g_npatch].orig = vtbl[slot];
    if (!VirtualProtect(&vtbl[slot], sizeof(void *), PAGE_EXECUTE_READWRITE, &old)) return NULL;
    vtbl[slot] = hook;
    VirtualProtect(&vtbl[slot], sizeof(void *), old, &old);
    return g_patch[g_npatch++].orig;
}

static void *orig_of(void *self, int slot)
{
    void **vtbl = *(void ***)self;
    int i;
    for (i = 0; i < g_npatch; i++)
        if (g_patch[i].vtbl == vtbl && g_patch[i].slot == slot) return g_patch[i].orig;
    return NULL;
}

/* ------------------------------------------------------------------ effect kinds */
static const char *SUB_NAME[12] = {
    "constant", "ramp", "square", "sine", "triangle", "saw_up", "saw_down",
    "spring", "damper", "inertia", "friction", "custom"
};
#define IS_PERIODIC(s) ((s) >= 2 && (s) <= 6)
#define IS_CONDITION(s) ((s) >= 7 && (s) <= 10)

static int fx_sub(const GUID *g)
{
    static const BYTE tail[8] = {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35};
    if (!g) return -1;
    if (g->Data2 == 0x8E33 && g->Data3 == 0x11D0 && !memcmp(g->Data4, tail, 8) &&
        g->Data1 >= 0x13541C20 && g->Data1 <= 0x13541C2B)
        return (int)(g->Data1 - 0x13541C20);
    return -1;
}

static const char *sub_name(int sub) { return sub >= 0 && sub < 12 ? SUB_NAME[sub] : "other"; }

static int pct_for(int sub)
{
    if (sub == 7) return g_cfg.spring;
    if (IS_CONDITION(sub)) return g_cfg.damper;
    if (IS_PERIODIC(sub)) return g_cfg.shake;
    if (sub == 0 || sub == 1) return g_cfg.constant;
    return 100;
}

/* ------------------------------------------------------------------ effect table */
#define TS_MAX 256
typedef struct {
    void *fx;
    int id, sub;
    char name[64];
    BYTE ts[TS_MAX];        /* last type-specific params exactly as Condor sent them */
    DWORD ts_len;
    DIENVELOPE env;
    int has_env;
    DWORD gain;
    unsigned sig;           /* signature of last logged SETP, to skip duplicates */
    int getp_logged;
    void *dev;              /* device the effect lives on */
    DIEFFECT base;          /* Condor's creation parameters, pointers re-aimed at copies below */
    DWORD axes[4];
    LONG dirs[4];
    int running;
    DWORD dev_gain, dev_mag;  /* periodic: gain and magnitude the device currently has */
    void *assist;             /* spring: our extra spring for boosts past the 100% cap */
    BYTE assist_ts[TS_MAX];
    DWORD assist_ts_len, assist_gain;
    int assist_fail;
    DWORD cgain;              /* last gain Condor itself set with SetParameters (not the creation default) */
} Fx;
static Fx g_fx[128];
static int g_nfx, g_next_id;

static Fx *fx_find(void *p)
{
    int i;
    for (i = 0; i < g_nfx; i++) if (g_fx[i].fx == p) return &g_fx[i];
    return NULL;
}

static Fx *fx_add(void *p)
{
    Fx *f = fx_find(p);
    if (!f) {
        if (g_nfx < (int)(sizeof g_fx / sizeof g_fx[0])) f = &g_fx[g_nfx++];
        else f = &g_fx[g_next_id % (int)(sizeof g_fx / sizeof g_fx[0])];
    }
    memset(f, 0, sizeof *f);
    f->fx = p;
    f->id = g_next_id++;
    return f;
}

static Fx *latest_named(const char *name);

/* wing-break residual: Condor cuts its VNE flutter dead when the wing goes; we keep some */
static struct { int active; void *obj; DWORD level; LONG k_break; DWORD t0; DWORD sent; void *zero; } g_brk;
/* linkage effects of our own: 0 friction, 1 inertia, 2 damper */
static struct { void *obj; void *dev; LONG sent; int fails; int started; } g_link[3];
static int g_link_restart, g_link_drop;
/* extra shake commanded by the pedal bridge (airbrake / high-speed buffet) */
static struct { void *obj; void *dev; DWORD want, period_ms, sent, sent_period; DWORD rx_tick; int fails, started; } g_buf;
/* gust forces from the bridge: constant forces on pitch (0) and roll (1) */
static struct { void *obj[2]; void *dev; LONG want[2], sent[2]; int fails, started[2]; } g_gust;
static SOCKET g_cmd = INVALID_SOCKET;
static int g_cmd_port_bound, g_buf_drop;

/* effects found in Effects.ffe, so the log can show Condor's own names */
typedef struct { int sub; DWORD period; char name[64]; } FileFx;
static FileFx g_filefx[32];
static int g_nfilefx;
static char g_pending_name[64];
static DWORD g_pending_tid;

/* ------------------------------------------------------------------ time, config, log, udp */
static long long unix_ms(void)
{
    typedef VOID (WINAPI *PFN)(LPFILETIME);
    static PFN precise;
    static int tried;
    FILETIME ft;
    ULARGE_INTEGER u;
    if (!tried) {
        HMODULE k = GetModuleHandleA("kernel32.dll");
        if (k) precise = (PFN)GetProcAddress(k, "GetSystemTimePreciseAsFileTime");
        tried = 1;
    }
    if (precise) precise(&ft); else GetSystemTimeAsFileTime(&ft);
    u.LowPart = ft.dwLowDateTime;
    u.HighPart = ft.dwHighDateTime;
    return (long long)((u.QuadPart - 116444736000000000ULL) / 10000ULL);
}

static void path_in_dir(char *out, const char *file)
{
    snprintf(out, MAX_PATH, "%s%s", g_dir, file);
}

static void open_log(void)
{
    char p[MAX_PATH];
    if (g_log || !g_cfg.log) return;
    path_in_dir(p, "ffb_shim.log");
    g_log = fopen(p, "ab");
    if (g_log) {
        fseek(g_log, 0, SEEK_END);
        g_log_bytes = ftell(g_log);
    }
}

static void udp_send(const char *s, int n)
{
    struct sockaddr_in a;
    if (g_cfg.udp_port <= 0) return;
    if (g_sock == INVALID_SOCKET) {
        WSADATA w;
        if (g_wsa_tried) return;
        g_wsa_tried = 1;
        if (WSAStartup(MAKEWORD(2, 2), &w) != 0) return;
        g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (g_sock == INVALID_SOCKET) return;
        {
            u_long nb = 1;
            ioctlsocket(g_sock, FIONBIO, &nb);
        }
    }
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((u_short)g_cfg.udp_port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sendto(g_sock, s, n, 0, (struct sockaddr *)&a, sizeof a);
}

/* kind: 0 = important event (always logged), 1 = SETP (subject to log_setp and size cap) */
static void emit(int kind, const char *fmt, ...)
{
    char buf[2048];
    int n, m;
    va_list ap;
    n = snprintf(buf, sizeof buf, "%lld ", unix_ms());
    va_start(ap, fmt);
    m = vsnprintf(buf + n, sizeof buf - n - 2, fmt, ap);
    va_end(ap);
    if (m < 0) return;
    n += m;
    if (n > (int)sizeof buf - 2) n = (int)sizeof buf - 2;
    udp_send(buf, n);
    if (kind == 2 || !g_cfg.log) return;
    open_log();
    if (!g_log) return;
    if (kind == 1) {
        if (!g_cfg.log_setp) return;
        if (g_log_bytes > (long long)g_cfg.max_mb * 1024 * 1024) {
            if (!g_log_capped) {
                g_log_capped = 1;
                fputs("--- log size cap reached: SETP lines no longer logged (still sent over UDP)\n", g_log);
            }
            return;
        }
    }
    buf[n++] = '\n';
    buf[n] = 0;
    fwrite(buf, 1, n, g_log);
    g_log_bytes += n;
    if (kind == 0 || GetTickCount() - g_last_flush > 1000) {
        fflush(g_log);
        g_last_flush = GetTickCount();
    }
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

static void read_cfg(int announce)
{
    char ini[MAX_PATH];
    Cfg c;
    path_in_dir(ini, "ffb_shim.ini");
    c.spring = clampi(GetPrivateProfileIntA("Gain", "spring", 100, ini), 0, 1000);
    c.damper = clampi(GetPrivateProfileIntA("Gain", "damper", 100, ini), 0, 1000);
    c.shake = clampi(GetPrivateProfileIntA("Gain", "shake", 100, ini), 0, 1000);
    c.constant = clampi(GetPrivateProfileIntA("Gain", "constant", 100, ini), 0, 1000);
    c.master = clampi(GetPrivateProfileIntA("Gain", "master", 100, ini), 0, 1000);
    c.spring_assist = GetPrivateProfileIntA("Gain", "spring_assist", 0, ini) != 0;
    c.log = GetPrivateProfileIntA("Log", "enabled", 1, ini) != 0;
    c.log_setp = GetPrivateProfileIntA("Log", "log_changes", 1, ini) != 0;
    c.max_mb = clampi(GetPrivateProfileIntA("Log", "max_mb", 50, ini), 1, 2000);
    c.udp_port = clampi(GetPrivateProfileIntA("Bridge", "udp_port", 55291, ini), 0, 65535);
    {
        char b[32];
        GetPrivateProfileStringA("Gain", "shake_curve", "1.0", b, sizeof b, ini);
        c.shake_curve = atof(b);
        if (c.shake_curve < 0.2 || c.shake_curve > 5.0) c.shake_curve = 1.0;
    }
    c.break_residual = clampi(GetPrivateProfileIntA("Break", "residual", 40, ini), 0, 100);
    c.link_friction = clampi(GetPrivateProfileIntA("Linkage", "friction", 0, ini), 0, 100);
    c.link_inertia = clampi(GetPrivateProfileIntA("Linkage", "inertia", 0, ini), 0, 100);
    c.link_damper = clampi(GetPrivateProfileIntA("Linkage", "damper", 0, ini), 0, 100);
    c.link_damper_speed = clampi(GetPrivateProfileIntA("Linkage", "damper_airspeed", 0, ini), 0, 100);
    c.cmd_port = clampi(GetPrivateProfileIntA("Bridge", "cmd_port", 55292, ini), 0, 65535);
    c.bridge_buffet = clampi(GetPrivateProfileIntA("Gain", "bridge_buffet", 100, ini), 0, 1000);
    c.bridge_gusts = clampi(GetPrivateProfileIntA("Gain", "bridge_gusts", 100, ini), 0, 1000);
    g_cfg = c;
    if (announce)
        emit(0, "CONFIG spring=%d spring_assist=%d shake=%d shake_curve=%.2f damper=%d constant=%d master=%d "
             "break_residual=%d linkage: friction=%d inertia=%d damper=%d damper_airspeed=%d log=%d log_changes=%d "
             "udp_port=%d cmd_port=%d bridge_buffet=%d bridge_gusts=%d", c.spring, c.spring_assist, c.shake,
             c.shake_curve, c.damper, c.constant, c.master, c.break_residual, c.link_friction, c.link_inertia,
             c.link_damper, c.link_damper_speed, c.log, c.log_setp, c.udp_port, c.cmd_port, c.bridge_buffet,
             c.bridge_gusts);
}

static void maybe_reload(void)
{
    WIN32_FILE_ATTRIBUTE_DATA a;
    char ini[MAX_PATH];
    DWORD now = GetTickCount();
    if (now - g_ini_check < 500) return;
    g_ini_check = now;
    path_in_dir(ini, "ffb_shim.ini");
    if (!GetFileAttributesExA(ini, GetFileExInfoStandard, &a)) return;
    if (CompareFileTime(&a.ftLastWriteTime, &g_ini_mtime) != 0) {
        g_ini_mtime = a.ftLastWriteTime;
        read_cfg(1);
    }
}

static int load_real(void)
{
    char p[MAX_PATH];
    if (p_DI8Create) return 1;
    GetSystemDirectoryA(p, MAX_PATH);   /* a 32-bit process is redirected to SysWOW64 */
    strncat(p, "\\dinput8.dll", MAX_PATH - strlen(p) - 1);
    g_real = LoadLibraryA(p);
    if (!g_real) return 0;
    p_DI8Create = (PFN_DI8Create)GetProcAddress(g_real, "DirectInput8Create");
    return p_DI8Create != NULL;
}

/* ------------------------------------------------------------------ formatting */
static int fmt_ts(char *o, int cap, int sub, const BYTE *ts, DWORD len)
{
    int n = 0;
    DWORD i;
    if (!ts || !len) return snprintf(o, cap, " ts=none");
    if (IS_CONDITION(sub)) {
        const DICONDITION *c = (const DICONDITION *)ts;
        for (i = 0; i < len / sizeof(DICONDITION) && n < cap - 1; i++)
            n += snprintf(o + n, cap - n, " a%lu:off=%ld,k+=%ld,k-=%ld,sat+=%lu,sat-=%lu,db=%ld",
                          (unsigned long)i, (long)c[i].lOffset, (long)c[i].lPositiveCoefficient,
                          (long)c[i].lNegativeCoefficient, (unsigned long)c[i].dwPositiveSaturation,
                          (unsigned long)c[i].dwNegativeSaturation, (long)c[i].lDeadBand);
    } else if (IS_PERIODIC(sub) && len >= sizeof(DIPERIODIC)) {
        const DIPERIODIC *p = (const DIPERIODIC *)ts;
        n = snprintf(o, cap, " mag=%lu off=%ld phase=%lu period_ms=%.1f",
                     (unsigned long)p->dwMagnitude, (long)p->lOffset, (unsigned long)p->dwPhase,
                     p->dwPeriod / 1000.0);
    } else if (sub == 0 && len >= sizeof(DICONSTANTFORCE)) {
        n = snprintf(o, cap, " mag=%ld", (long)((const DICONSTANTFORCE *)ts)->lMagnitude);
    } else if (sub == 1 && len >= sizeof(DIRAMPFORCE)) {
        const DIRAMPFORCE *r = (const DIRAMPFORCE *)ts;
        n = snprintf(o, cap, " start=%ld end=%ld", (long)r->lStart, (long)r->lEnd);
    } else {
        n = snprintf(o, cap, " ts_bytes=%lu", (unsigned long)len);
    }
    return n < cap ? n : cap - 1;
}

static int fmt_common(char *o, int cap, LPCDIEFFECT e, DWORD flags)
{
    int n = 0;
    DWORD i;
    if (flags & DIEP_GAIN) n += snprintf(o + n, cap - n, " gain=%lu", (unsigned long)e->dwGain);
    if (flags & DIEP_DURATION) {
        if (e->dwDuration == INFINITE) n += snprintf(o + n, cap - n, " dur=inf");
        else n += snprintf(o + n, cap - n, " dur_ms=%lu", (unsigned long)(e->dwDuration / 1000));
    }
    if ((flags & DIEP_AXES) && e->rgdwAxes && n < cap - 1) {
        n += snprintf(o + n, cap - n, " axes=");
        for (i = 0; i < e->cAxes && i < 4 && n < cap - 1; i++)
            n += snprintf(o + n, cap - n, "%s%lu", i ? "," : "", (unsigned long)e->rgdwAxes[i]);
    }
    if ((flags & DIEP_DIRECTION) && e->rglDirection && n < cap - 1) {
        n += snprintf(o + n, cap - n, " dir=");
        for (i = 0; i < e->cAxes && i < 4 && n < cap - 1; i++)
            n += snprintf(o + n, cap - n, "%s%ld", i ? "," : "", (long)e->rglDirection[i]);
    }
    if ((flags & DIEP_ENVELOPE) && e->lpEnvelope && n < cap - 1) {
        const DIENVELOPE *v = e->lpEnvelope;
        n += snprintf(o + n, cap - n, " env=%lu/%lums,%lu/%lums", (unsigned long)v->dwAttackLevel,
                      (unsigned long)(v->dwAttackTime / 1000), (unsigned long)v->dwFadeLevel,
                      (unsigned long)(v->dwFadeTime / 1000));
    }
    if (flags & DIEP_START) n += snprintf(o + n, cap - n, " +START");
    return n < cap ? n : cap - 1;
}

/* ------------------------------------------------------------------ scaling */
static LONG scl(LONG v, int pct, LONG lim, int *clip)
{
    long long r = (long long)v * pct / 100;
    if (r > lim) { r = lim; *clip = 1; }
    if (r < -lim) { r = -lim; *clip = 1; }
    return (LONG)r;
}

typedef struct { DIEFFECT e; BYTE ts[TS_MAX]; DIENVELOPE env; DWORD flags; } Scaled;

/* Spring boost that tapers: k' = 10000 * (1 - (1 - k/10000)^(pct/100)). Slope at zero is
 * pct/100 (150% = half as stiff again at low speed), and Condor's top stiffness (10000, ~97 kt)
 * still maps to 10000 - so the boosted spring never asks for more than the stick's maximum
 * before the end of the travel. */
static LONG soft_boost(LONG k, int pct)
{
    double x = (k < 0 ? -k : k) / 10000.0, y;
    if (x > 1.0) x = 1.0;
    y = 1.0 - pow(1.0 - x, pct / 100.0);
    return (LONG)((k < 0 ? -y : y) * 10000.0 + (k < 0 ? -0.5 : 0.5));
}

static int copy_effect(LPCDIEFFECT in, Scaled *out, DWORD flags)
{
    DWORD sz;
    if (!in) return 0;
    sz = in->dwSize;
    if (sz < sizeof(DIEFFECT_DX5) || sz > sizeof(DIEFFECT)) return 0;
    memset(out, 0, sizeof *out);
    memcpy(&out->e, in, sz);
    out->flags = flags;
    return 1;
}

/* Spring / damper / constant / ramp: scale the type-specific strengths. Returns 1 if changed. */
static int make_scaled(int sub, LPCDIEFFECT in, DWORD flags, Scaled *out, int *clip)
{
    int pct = pct_for(sub), changed = 0;
    DWORD i;
    *clip = 0;
    if (!in || pct == 100 || sub < 0 || IS_PERIODIC(sub)) return 0;
    if (!copy_effect(in, out, flags)) return 0;
    if ((flags & DIEP_TYPESPECIFICPARAMS) && in->lpvTypeSpecificParams &&
        in->cbTypeSpecificParams > 0 && in->cbTypeSpecificParams <= TS_MAX) {
        memcpy(out->ts, in->lpvTypeSpecificParams, in->cbTypeSpecificParams);
        if (IS_CONDITION(sub)) {
            DICONDITION *c = (DICONDITION *)out->ts;
            for (i = 0; i < in->cbTypeSpecificParams / sizeof(DICONDITION); i++) {
                if (sub == 7 && pct > 100 && !g_cfg.spring_assist) {
                    c[i].lPositiveCoefficient = soft_boost(c[i].lPositiveCoefficient, pct);
                    c[i].lNegativeCoefficient = soft_boost(c[i].lNegativeCoefficient, pct);
                    continue;      /* saturation stays as Condor set it */
                }
                c[i].lPositiveCoefficient = scl(c[i].lPositiveCoefficient, pct, 10000, clip);
                c[i].lNegativeCoefficient = scl(c[i].lNegativeCoefficient, pct, 10000, clip);
                if (pct > 100) {   /* let the stronger spring actually reach its new force */
                    int dummy = 0;
                    if (c[i].dwPositiveSaturation)
                        c[i].dwPositiveSaturation = (DWORD)scl((LONG)c[i].dwPositiveSaturation, pct, 10000, &dummy);
                    if (c[i].dwNegativeSaturation)
                        c[i].dwNegativeSaturation = (DWORD)scl((LONG)c[i].dwNegativeSaturation, pct, 10000, &dummy);
                }
            }
        } else if (sub == 0 && in->cbTypeSpecificParams >= sizeof(DICONSTANTFORCE)) {
            DICONSTANTFORCE *k = (DICONSTANTFORCE *)out->ts;
            k->lMagnitude = scl(k->lMagnitude, pct, 10000, clip);
        } else if (sub == 1 && in->cbTypeSpecificParams >= sizeof(DIRAMPFORCE)) {
            DIRAMPFORCE *r = (DIRAMPFORCE *)out->ts;
            r->lStart = scl(r->lStart, pct, 10000, clip);
            r->lEnd = scl(r->lEnd, pct, 10000, clip);
        }
        out->e.lpvTypeSpecificParams = out->ts;
        changed = 1;
    }
    if ((flags & DIEP_ENVELOPE) && in->lpEnvelope && in->lpEnvelope->dwSize >= sizeof(DIENVELOPE) &&
        !IS_CONDITION(sub)) {
        int c2 = 0;
        out->env = *in->lpEnvelope;
        out->env.dwAttackLevel = (DWORD)scl((LONG)out->env.dwAttackLevel, pct, 10000, &c2);
        out->env.dwFadeLevel = (DWORD)scl((LONG)out->env.dwFadeLevel, pct, 10000, &c2);
        out->e.lpEnvelope = &out->env;
        changed = 1;
    }
    return changed;
}

/* Periodic (Condor's stall / VNE shake): Condor sets the intensity through the effect gain
 * (0-10000) on a fixed-magnitude triangle, so scale the gain. Once the gain hits 10000 the
 * rest spills into the magnitude. Adds DIEP_GAIN / DIEP_TYPESPECIFICPARAMS as needed so the
 * device always holds gain' x magnitude' = Condor's gain x magnitude x shake%. */
static int scale_periodic(Fx *f, LPCDIEFFECT in, DWORD flags, Scaled *out, DWORD *g_out, DWORD *m_out, int *clip)
{
    int pct = g_cfg.shake;
    DWORD base_gain, base_mag, g2, m2;
    double want, m;
    const DIPERIODIC *src;
    *clip = 0;
    if (!f || !in) return 0;
    src = ((flags & DIEP_TYPESPECIFICPARAMS) && in->lpvTypeSpecificParams &&
           in->cbTypeSpecificParams >= sizeof(DIPERIODIC)) ? (const DIPERIODIC *)in->lpvTypeSpecificParams
          : (f->ts_len >= sizeof(DIPERIODIC) ? (const DIPERIODIC *)f->ts : NULL);
    if (!src) return 0;
    base_gain = (flags & DIEP_GAIN) ? in->dwGain : f->gain;
    base_mag = src->dwMagnitude;
    want = 10000.0 * pow(base_gain / 10000.0, g_cfg.shake_curve) * (double)pct / 100.0;
    g2 = want >= 10000.0 ? 10000 : (DWORD)(want + 0.5);
    m = base_mag * (g2 > 0 && want > 10000.0 ? want / g2 : 1.0);
    m2 = m >= 10000.0 ? 10000 : (DWORD)(m + 0.5);
    if (want > 10000.0 && m > 10000.0) *clip = 1;
    if (!copy_effect(in, out, flags)) return 0;
    if ((flags & DIEP_GAIN) || g2 != f->dev_gain) {
        out->e.dwGain = g2;
        out->flags |= DIEP_GAIN;
    }
    if ((flags & DIEP_TYPESPECIFICPARAMS) || m2 != f->dev_mag) {
        DWORD cb = (flags & DIEP_TYPESPECIFICPARAMS) ? in->cbTypeSpecificParams : f->ts_len;
        if (cb > TS_MAX) cb = TS_MAX;
        memcpy(out->ts, src, cb);
        ((DIPERIODIC *)out->ts)->dwMagnitude = m2;
        out->e.cbTypeSpecificParams = cb;
        out->e.lpvTypeSpecificParams = out->ts;
        out->flags |= DIEP_TYPESPECIFICPARAMS;
    }
    *g_out = g2;
    *m_out = m2;
    return out->flags != flags || ((flags & DIEP_GAIN) && g2 != base_gain) ||
           ((flags & DIEP_TYPESPECIFICPARAMS) && m2 != base_mag);
}

/* Spring assist: per axis, whatever the boosted stiffness has above 10000 goes to the
 * assist spring. Returns the byte length of the assist params (0 = no assist needed). */
static DWORD assist_params(Fx *f, BYTE *out)
{
    DWORD i, n;
    int any = 0;
    const DICONDITION *c = (const DICONDITION *)f->ts;
    DICONDITION *a = (DICONDITION *)out;
    if (f->sub != 7 || f->ts_len < sizeof(DICONDITION)) return 0;
    if (!g_cfg.spring_assist && !f->assist) return 0;
    n = f->ts_len / sizeof(DICONDITION);
    for (i = 0; i < n; i++) {
        long long kp = (long long)c[i].lPositiveCoefficient * g_cfg.spring / 100;
        long long kn = (long long)c[i].lNegativeCoefficient * g_cfg.spring / 100;
        a[i] = c[i];
        a[i].lPositiveCoefficient = (LONG)(kp > 10000 ? (kp - 10000 > 10000 ? 10000 : kp - 10000)
                                           : (kp < -10000 ? (kp + 10000 < -10000 ? -10000 : kp + 10000) : 0));
        a[i].lNegativeCoefficient = (LONG)(kn > 10000 ? (kn - 10000 > 10000 ? 10000 : kn - 10000)
                                           : (kn < -10000 ? (kn + 10000 < -10000 ? -10000 : kn + 10000) : 0));
        /* same force ceiling as the boosted main spring (0 stays 0: no force) */
        a[i].dwPositiveSaturation = (DWORD)(c[i].dwPositiveSaturation * (long long)g_cfg.spring / 100 > 10000 ? 10000
                                            : c[i].dwPositiveSaturation * (long long)g_cfg.spring / 100);
        a[i].dwNegativeSaturation = (DWORD)(c[i].dwNegativeSaturation * (long long)g_cfg.spring / 100 > 10000 ? 10000
                                            : c[i].dwNegativeSaturation * (long long)g_cfg.spring / 100);
        if ((a[i].dwPositiveSaturation == 0 && a[i].dwNegativeSaturation == 0) || !g_cfg.spring_assist) {
            a[i].lPositiveCoefficient = a[i].lNegativeCoefficient = 0;
        }
        if (a[i].lPositiveCoefficient || a[i].lNegativeCoefficient) any = 1;
    }
    if (!any && !f->assist) return 0;
    return n * sizeof(DICONDITION);
}

static void remember(Fx *f, LPCDIEFFECT in, DWORD flags)
{
    if (!f || !in) return;
    if ((flags & DIEP_TYPESPECIFICPARAMS) && in->lpvTypeSpecificParams &&
        in->cbTypeSpecificParams <= TS_MAX) {
        memcpy(f->ts, in->lpvTypeSpecificParams, in->cbTypeSpecificParams);
        f->ts_len = in->cbTypeSpecificParams;
    }
    if ((flags & DIEP_ENVELOPE) && in->lpEnvelope) {
        f->env = *in->lpEnvelope;
        f->has_env = 1;
    }
    if (flags & DIEP_GAIN) f->gain = in->dwGain;
}

static unsigned sig_of(LPCDIEFFECT in, DWORD flags)
{
    unsigned h = 2166136261u ^ flags;
    DWORD i;
    if (!in) return h;
    if (flags & DIEP_GAIN) h = (h ^ in->dwGain) * 16777619u;
    if ((flags & DIEP_TYPESPECIFICPARAMS) && in->lpvTypeSpecificParams && in->cbTypeSpecificParams <= TS_MAX)
        for (i = 0; i < in->cbTypeSpecificParams; i++)
            h = (h ^ ((const BYTE *)in->lpvTypeSpecificParams)[i]) * 16777619u;
    if ((flags & DIEP_DIRECTION) && in->rglDirection)
        for (i = 0; i < in->cAxes && i < 4; i++) h = (h ^ (unsigned)in->rglDirection[i]) * 16777619u;
    return h;
}

/* ------------------------------------------------------------------ effect hooks */
typedef HRESULT (WINAPI *PFN_GetParameters)(void *, LPDIEFFECT, DWORD);
typedef HRESULT (WINAPI *PFN_SetParameters)(void *, LPCDIEFFECT, DWORD);
typedef HRESULT (WINAPI *PFN_Start)(void *, DWORD, DWORD);
typedef HRESULT (WINAPI *PFN_Stop)(void *);
typedef ULONG (WINAPI *PFN_Release)(void *);
typedef HRESULT (WINAPI *PFN_CreateEffect)(void *, REFGUID, LPCDIEFFECT, LPDIRECTINPUTEFFECT *, LPUNKNOWN);

static const GUID GUID_Spring_ = {0x13541C27, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};

/* Bring the assist spring in line with Condor's spring. Called WITHOUT g_cs held. */
static void sync_assist(Fx *f)
{
    BYTE ts[TS_MAX];
    DWORD len;
    void *assist, *dev;
    int running, create = 0, update = 0;
    DWORD gain, uflags = 0;
    DIEFFECT e;
    HRESULT hr = DI_OK;
    EnterCriticalSection(&g_cs);
    len = assist_params(f, ts);
    assist = f->assist;
    dev = f->dev;
    running = f->running;
    gain = f->gain;
    if (len) {
        if (!assist && f->assist_fail < 3 && dev) create = 1;
        else if (assist) {
            if (len != f->assist_ts_len || memcmp(ts, f->assist_ts, len)) uflags |= DIEP_TYPESPECIFICPARAMS;
            if (gain != f->assist_gain) uflags |= DIEP_GAIN;
            update = uflags != 0;
        }
    }
    e = f->base;
    e.dwSize = sizeof(DIEFFECT);
    e.rgdwAxes = f->axes;
    e.rglDirection = f->dirs;
    e.lpEnvelope = NULL;
    e.dwGain = gain;
    e.cbTypeSpecificParams = len;
    e.lpvTypeSpecificParams = ts;
    LeaveCriticalSection(&g_cs);
    if (create) {
        PFN_CreateEffect ce = (PFN_CreateEffect)orig_of(dev, DEV_CreateEffect);
        LPDIRECTINPUTEFFECT a = NULL;
        hr = ce ? ce(dev, &GUID_Spring_, &e, &a, NULL) : DIERR_GENERIC;
        if (SUCCEEDED(hr) && a && running) {
            PFN_Start st = (PFN_Start)orig_of(a, FX_Start);
            if (st) st(a, 1, 0);
        }
        EnterCriticalSection(&g_cs);
        if (SUCCEEDED(hr) && a) {
            f->assist = a;
            memcpy(f->assist_ts, ts, len);
            f->assist_ts_len = len;
            f->assist_gain = gain;
            emit(0, "ASSIST created for fx=%d %s k+=%ld (running=%d)", f->id, f->name,
                 (long)((DICONDITION *)ts)->lPositiveCoefficient, running);
        } else {
            f->assist_fail++;
            emit(0, "ASSIST create FAILED for fx=%d hr=0x%08lx (try %d of 3)", f->id, (unsigned long)hr,
                 f->assist_fail);
        }
        LeaveCriticalSection(&g_cs);
    } else if (update) {
        PFN_SetParameters sp = (PFN_SetParameters)orig_of(assist, FX_SetParameters);
        hr = sp ? sp(assist, &e, uflags) : DIERR_GENERIC;
        EnterCriticalSection(&g_cs);
        if (SUCCEEDED(hr)) {
            memcpy(f->assist_ts, ts, len);
            f->assist_ts_len = len;
            f->assist_gain = gain;
        } else {
            emit(0, "ASSIST update FAILED for fx=%d hr=0x%08lx", f->id, (unsigned long)hr);
        }
        LeaveCriticalSection(&g_cs);
    }
}

/* --- wing break helpers (call with g_cs held) */
static LONG spring_k_now(void)
{
    Fx *sp = latest_named("Spring");
    return sp && sp->ts_len >= sizeof(DICONDITION) ? ((DICONDITION *)sp->ts)->lPositiveCoefficient : -1;
}

static DWORD residual_gain(void)
{
    LONG k = spring_k_now();
    double frac = (g_brk.k_break > 0 && k >= 0) ? (double)k / g_brk.k_break : 0.0;
    if (frac > 1.0) frac = 1.0;
    if (frac < 0.0) frac = 0.0;
    return (DWORD)(g_brk.level * g_cfg.break_residual / 100.0 * frac + 0.5);
}

static void brk_clear(const char *why, int send_zero)
{
    if (!g_brk.active) return;
    if (send_zero && g_brk.sent) g_brk.zero = g_brk.obj;
    g_brk.active = 0;
    emit(0, "BREAK residual off (%s)", why);
}

static HRESULT WINAPI Hook_SetParameters(void *self, LPCDIEFFECT peff, DWORD flags)
{
    PFN_SetParameters orig;
    Fx *f;
    Scaled s;
    int clip = 0, use = 0, periodic = 0;
    DWORD g2 = 0, m2 = 0;
    HRESULT hr;
    char line[1500];
    int n;
    EnterCriticalSection(&g_cs);
    maybe_reload();
    g_setp_calls++;
    orig = (PFN_SetParameters)orig_of(self, FX_SetParameters);
    f = fx_find(self);
    if (f && peff) {
        periodic = IS_PERIODIC(f->sub);
        if (periodic && (flags & DIEP_GAIN)) {
            Fx *s1 = latest_named("Stall1");
            DWORD s1g = (s1 && s1 != f) ? s1->gain : 0;
            if (g_brk.active && peff->dwGain > 0) {
                brk_clear("Condor is shaking the stick again", 0);
            } else if (!g_brk.active && g_cfg.break_residual > 0 && !strcmp(f->name, "Stall2") &&
                       f->cgain >= 3000 && peff->dwGain == 0 && s1g < 50) {
                g_brk.active = 1;
                g_brk.obj = self;
                g_brk.level = f->dev_gain;
                g_brk.k_break = spring_k_now();
                g_brk.t0 = GetTickCount();
                g_brk.sent = 0;
                g_brk.zero = NULL;
                emit(0, "BREAK Condor cut its VNE flutter dead (gain %lu -> 0): keeping %d%% of it, fading with "
                     "airspeed (spring k %ld)", (unsigned long)f->cgain, g_cfg.break_residual, (long)g_brk.k_break);
            }
            f->cgain = peff->dwGain;
            if (g_brk.active && self == g_brk.obj && peff->dwGain == 0 && copy_effect(peff, &s, flags)) {
                g2 = residual_gain();
                m2 = f->dev_mag;
                s.e.dwGain = g2;
                s.flags = flags | DIEP_GAIN;
                g_brk.sent = g2;
                use = 1;
            }
        }
        if (!use)
            use = periodic ? scale_periodic(f, peff, flags, &s, &g2, &m2, &clip)
                           : make_scaled(f->sub, peff, flags, &s, &clip);
        if (!use) s.flags = flags;
    }
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, use ? &s.e : peff, use ? s.flags : flags);
    EnterCriticalSection(&g_cs);
    if (f && peff) {
        unsigned sig = sig_of(peff, flags);
        remember(f, peff, flags);
        if (periodic && use && SUCCEEDED(hr)) {
            if (s.flags & DIEP_GAIN) f->dev_gain = g2;
            if (s.flags & DIEP_TYPESPECIFICPARAMS) f->dev_mag = m2;
        }
        if (sig != f->sig || FAILED(hr) || (flags & DIEP_START)) {
            f->sig = sig;
            n = snprintf(line, sizeof line, "SETP fx=%d %s %s flags=0x%lx", f->id, f->name,
                         sub_name(f->sub), (unsigned long)flags);
            n += fmt_common(line + n, sizeof line - n, peff, flags);
            if (flags & DIEP_TYPESPECIFICPARAMS)
                n += fmt_ts(line + n, sizeof line - n, f->sub, (const BYTE *)peff->lpvTypeSpecificParams,
                            peff->cbTypeSpecificParams);
            if (use && periodic)
                n += snprintf(line + n, sizeof line - n, " -> gain=%lu mag=%lu (x%d%%)%s", (unsigned long)g2,
                              (unsigned long)m2, g_cfg.shake, clip ? " CLIPPED" : "");
            else if (use)
                n += snprintf(line + n, sizeof line - n, " x%d%%%s", pct_for(f->sub), clip ? " CLIPPED" : "");
            if (FAILED(hr)) n += snprintf(line + n, sizeof line - n, " hr=0x%08lx", (unsigned long)hr);
            emit((flags & DIEP_START) || FAILED(hr) ? 0 : 1, "%s", line);
        }
    }
    LeaveCriticalSection(&g_cs);
    if (f && f->sub == 7 && SUCCEEDED(hr)) sync_assist(f);
    return hr;
}

static HRESULT WINAPI Hook_GetParameters(void *self, LPDIEFFECT peff, DWORD flags)
{
    PFN_GetParameters orig;
    HRESULT hr;
    Fx *f;
    EnterCriticalSection(&g_cs);
    orig = (PFN_GetParameters)orig_of(self, FX_GetParameters);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, peff, flags);
    if (FAILED(hr) || !peff) return hr;
    EnterCriticalSection(&g_cs);
    f = fx_find(self);
    if (f) {
        /* hand back Condor's own numbers, so a read-modify-write never compounds our boost */
        if ((flags & DIEP_TYPESPECIFICPARAMS) && peff->lpvTypeSpecificParams && f->ts_len &&
            peff->cbTypeSpecificParams == f->ts_len)
            memcpy(peff->lpvTypeSpecificParams, f->ts, f->ts_len);
        if ((flags & DIEP_GAIN) && IS_PERIODIC(f->sub)) peff->dwGain = f->gain;
        if ((flags & DIEP_ENVELOPE) && peff->lpEnvelope && f->has_env) {
            peff->lpEnvelope->dwAttackLevel = f->env.dwAttackLevel;
            peff->lpEnvelope->dwFadeLevel = f->env.dwFadeLevel;
        }
        if (f->getp_logged < 3) {
            f->getp_logged++;
            emit(0, "GETP fx=%d %s flags=0x%lx", f->id, f->name, (unsigned long)flags);
        }
    }
    LeaveCriticalSection(&g_cs);
    return hr;
}

static HRESULT WINAPI Hook_Start(void *self, DWORD iter, DWORD flags)
{
    PFN_Start orig;
    HRESULT hr;
    Fx *f;
    void *assist = NULL;
    EnterCriticalSection(&g_cs);
    orig = (PFN_Start)orig_of(self, FX_Start);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, iter, flags);
    EnterCriticalSection(&g_cs);
    f = fx_find(self);
    if (f && SUCCEEDED(hr)) {
        f->running = 1;
        assist = f->assist;
        if (f->sub == 7) {
            g_link_restart = 1;
            g_buf.started = 0;
            g_gust.started[0] = g_gust.started[1] = 0;
        }
    }
    emit(0, "START fx=%d %s %s iter=%lu flags=0x%lx hr=0x%08lx", f ? f->id : -1, f ? f->name : "?",
         sub_name(f ? f->sub : -1), (unsigned long)iter, (unsigned long)flags, (unsigned long)hr);
    LeaveCriticalSection(&g_cs);
    if (assist) orig(assist, iter, flags & ~DIES_NODOWNLOAD);
    return hr;
}

static HRESULT WINAPI Hook_Stop(void *self)
{
    PFN_Stop orig;
    HRESULT hr;
    Fx *f;
    void *assist = NULL;
    EnterCriticalSection(&g_cs);
    orig = (PFN_Stop)orig_of(self, FX_Stop);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self);
    EnterCriticalSection(&g_cs);
    f = fx_find(self);
    if (f) {
        f->running = 0;
        assist = f->assist;
    }
    emit(0, "STOP fx=%d %s %s hr=0x%08lx", f ? f->id : -1, f ? f->name : "?", sub_name(f ? f->sub : -1),
         (unsigned long)hr);
    LeaveCriticalSection(&g_cs);
    if (assist) orig(assist);
    return hr;
}

static ULONG WINAPI Hook_Release(void *self)
{
    PFN_Release orig;
    ULONG left;
    void *assist = NULL;
    Fx *f;
    EnterCriticalSection(&g_cs);
    orig = (PFN_Release)orig_of(self, FX_Release);
    LeaveCriticalSection(&g_cs);
    if (!orig) return 0;
    left = orig(self);
    if (left == 0) {
        EnterCriticalSection(&g_cs);
        f = fx_find(self);
        if (f) {
            assist = f->assist;
            f->assist = NULL;
            emit(0, "RELEASE fx=%d %s", f->id, f->name);
            f->fx = NULL;
        }
        LeaveCriticalSection(&g_cs);
        if (assist) orig(assist);
    }
    return left;
}

/* Our assist springs die with a device reset; forget them so they get recreated. */
static void drop_assists(void *dev)
{
    void *dead[16];
    int i, nd = 0;
    PFN_Release rel = NULL;
    EnterCriticalSection(&g_cs);
    for (i = 0; i < g_nfx && nd < 16; i++)
        if (g_fx[i].fx && g_fx[i].assist && g_fx[i].dev == dev) {
            dead[nd++] = g_fx[i].assist;
            g_fx[i].assist = NULL;
            g_fx[i].assist_ts_len = 0;
        }
    if (nd) rel = (PFN_Release)orig_of(dead[0], FX_Release);
    g_link_drop = 1;                  /* the heartbeat thread lets go of the linkage effects */
    g_buf_drop = 1;
    LeaveCriticalSection(&g_cs);
    for (i = 0; i < nd; i++) if (rel) rel(dead[i]);
}

/* Heartbeat for the pedal bridge: Condor's current shake gains and spring stiffness. */
static Fx *latest_named(const char *name)
{
    int i;
    Fx *best = NULL;
    for (i = 0; i < g_nfx; i++)
        if (g_fx[i].fx && !strcmp(g_fx[i].name, name) && (!best || g_fx[i].id > best->id)) best = &g_fx[i];
    return best;
}

static const GUID GUID_Friction_ = {0x13541C2A, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
static const GUID GUID_Inertia_ = {0x13541C29, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
static const GUID GUID_Damper_ = {0x13541C28, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
static const char *LINK_NAME[3] = {"friction", "inertia", "damper"};

static void set_gain_now(void *fx, DWORD gain)
{
    DIEFFECT e;
    PFN_SetParameters sp = (PFN_SetParameters)orig_of(fx, FX_SetParameters);
    memset(&e, 0, sizeof e);
    e.dwSize = sizeof e;
    e.dwGain = gain;
    if (sp) sp(fx, &e, DIEP_GAIN);
}

/* Bridge commands: "BUFFET gain=<0-10000> period_ms=<n>", about 20 per second. */
static void poll_commands(void)
{
    char b[256];
    int n;
    if (g_cfg.cmd_port <= 0) return;
    if (g_cmd == INVALID_SOCKET || g_cmd_port_bound != g_cfg.cmd_port) {
        struct sockaddr_in a;
        WSADATA w;
        u_long nb = 1;
        if (g_cmd != INVALID_SOCKET) closesocket(g_cmd);
        g_cmd = INVALID_SOCKET;
        g_cmd_port_bound = g_cfg.cmd_port;
        WSAStartup(MAKEWORD(2, 2), &w);
        g_cmd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (g_cmd == INVALID_SOCKET) return;
        memset(&a, 0, sizeof a);
        a.sin_family = AF_INET;
        a.sin_port = htons((u_short)g_cfg.cmd_port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(g_cmd, (struct sockaddr *)&a, sizeof a) != 0) {
            EnterCriticalSection(&g_cs);
            emit(0, "BRIDGE can't listen on UDP %d - no bridge buffet on the stick", g_cfg.cmd_port);
            LeaveCriticalSection(&g_cs);
            closesocket(g_cmd);
            g_cmd = INVALID_SOCKET;
            return;
        }
        ioctlsocket(g_cmd, FIONBIO, &nb);
    }
    while ((n = recv(g_cmd, b, sizeof b - 1, 0)) > 0) {
        char *p;
        b[n] = 0;
        if (strncmp(b, "BUFFET", 6)) continue;
        EnterCriticalSection(&g_cs);
        g_buf.want = 0;
        if ((p = strstr(b, "gain="))) g_buf.want = (DWORD)clampi(atoi(p + 5), 0, 10000);
        if ((p = strstr(b, "period_ms="))) g_buf.period_ms = (DWORD)clampi(atoi(p + 10), 10, 1000);
        g_gust.want[0] = (p = strstr(b, "pitch=")) ? clampi(atoi(p + 6), -10000, 10000) : 0;
        g_gust.want[1] = (p = strstr(b, "roll=")) ? clampi(atoi(p + 5), -10000, 10000) : 0;
        if (!g_buf.period_ms) g_buf.period_ms = 70;
        g_buf.rx_tick = GetTickCount();
        LeaveCriticalSection(&g_cs);
    }
}

static const GUID GUID_Triangle_ = {0x13541C24, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};

/* Plays the bridge's buffet as a triangle wave of our own on the stick (pitch, like Condor's
 * Stall1). Goes quiet if the bridge stops sending for half a second. */
static void bridge_buffet(void)
{
    Fx *ref, *sp;
    void *dev = NULL, *drop = NULL, *obj;
    DWORD want, period, a;
    int act = 0, running = 0;          /* 1 create, 2 update */
    DIEFFECT e;
    DWORD axes[4];
    LONG dirs[4];
    DIPERIODIC per;
    HRESULT hr;

    poll_commands();
    EnterCriticalSection(&g_cs);
    want = (GetTickCount() - g_buf.rx_tick < 500) ? g_buf.want : 0;
    want = (DWORD)((long long)want * g_cfg.bridge_buffet / 100);
    if (want > 10000) want = 10000;
    period = g_buf.period_ms ? g_buf.period_ms : 70;
    sp = latest_named("Spring");
    ref = latest_named("Stall1");
    if (!ref) ref = sp;
    if (sp) running = sp->running;
    if (ref && ref->dev && ref->base.cAxes > 0) {
        dev = ref->dev;
        e = ref->base;
        memcpy(axes, ref->axes, sizeof axes);
        memcpy(dirs, ref->dirs, sizeof dirs);
        if (ref == sp) for (a = 0; a < 4; a++) dirs[a] = 0;
    }
    if (g_buf.obj && (g_buf.dev != dev || g_buf_drop)) {
        drop = g_buf.obj;
        g_buf.obj = NULL;
        g_buf.sent = 0;
        g_buf.started = 0;
    }
    if (dev) {
        if (!g_buf.obj && want > 0 && running && g_buf.fails < 3) act = 1;
        else if (g_buf.obj && ((want > g_buf.sent ? want - g_buf.sent : g_buf.sent - want) >= 50 ||
                               (want == 0 && g_buf.sent) || period != g_buf.sent_period))
            act = 2;
    }
    obj = g_buf.obj;
    LeaveCriticalSection(&g_cs);

    if (drop) {
        PFN_Release rel = (PFN_Release)orig_of(drop, FX_Release);
        if (rel) rel(drop);
    }
    if (!act) return;
    memset(&per, 0, sizeof per);
    per.dwMagnitude = 10000;
    per.dwPeriod = period * 1000;
    e.dwSize = sizeof(DIEFFECT);
    e.rgdwAxes = axes;
    e.rglDirection = dirs;
    e.lpEnvelope = NULL;
    e.dwGain = want;
    e.dwDuration = INFINITE;
    e.dwTriggerButton = DIEB_NOTRIGGER;
    e.cbTypeSpecificParams = sizeof per;
    e.lpvTypeSpecificParams = &per;
    if (act == 1) {
        PFN_CreateEffect ce = (PFN_CreateEffect)orig_of(dev, DEV_CreateEffect);
        LPDIRECTINPUTEFFECT o = NULL;
        HRESULT h2 = DIERR_GENERIC;
        hr = ce ? ce(dev, &GUID_Triangle_, &e, &o, NULL) : DIERR_GENERIC;
        if (SUCCEEDED(hr) && o) {
            PFN_Start st = (PFN_Start)orig_of(o, FX_Start);
            h2 = st ? st(o, 1, 0) : DIERR_GENERIC;
        }
        EnterCriticalSection(&g_cs);
        if (SUCCEEDED(hr) && o) {
            g_buf.obj = o;
            g_buf.dev = dev;
            g_buf.sent = want;
            g_buf.sent_period = period;
            g_buf.started = SUCCEEDED(h2);
            emit(0, "BUFFET effect created (gain %lu, %lu ms, start hr=0x%08lx)", (unsigned long)want,
                 (unsigned long)period, (unsigned long)h2);
        } else {
            g_buf.fails++;
            emit(0, "BUFFET create FAILED hr=0x%08lx (try %d of 3)", (unsigned long)hr, g_buf.fails);
        }
        LeaveCriticalSection(&g_cs);
    } else {
        PFN_SetParameters spf = (PFN_SetParameters)orig_of(obj, FX_SetParameters);
        hr = spf ? spf(obj, &e, DIEP_GAIN | DIEP_TYPESPECIFICPARAMS) : DIERR_GENERIC;
        EnterCriticalSection(&g_cs);
        if (SUCCEEDED(hr)) {
            g_buf.sent = want;
            g_buf.sent_period = period;
        }
        LeaveCriticalSection(&g_cs);
        if (SUCCEEDED(hr) && !g_buf.started && running) {
            PFN_Start st = (PFN_Start)orig_of(obj, FX_Start);
            if (st && SUCCEEDED(st(obj, 1, 0))) g_buf.started = 1;
        }
    }
}

static const GUID GUID_ConstantForce_ = {0x13541C20, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};

/* Gust forces: one single-axis constant force per stick axis. pitch > 0 pushes the stick
 * forward, roll > 0 pushes it right (DirectInput forces point away from where they come
 * from, so the roll magnitude is negated). Silent 0.5 s after the bridge stops sending. */
static void gust_forces(void)
{
    Fx *sp;
    void *dev = NULL, *drop[2] = {NULL, NULL}, *obj[2];
    DWORD axis[2] = {0, 0};
    LONG want[2], sent[2];
    int i, act[2] = {0, 0}, running = 0, fresh;
    EnterCriticalSection(&g_cs);
    fresh = GetTickCount() - g_buf.rx_tick < 500;
    sp = latest_named("Spring");
    if (sp && sp->dev && sp->base.cAxes >= 2) {
        dev = sp->dev;
        running = sp->running;
        axis[0] = sp->axes[1];         /* Condor's spring: axes 0 = roll (X), 1 = pitch (Y) */
        axis[1] = sp->axes[0];
    }
    for (i = 0; i < 2; i++) {
        want[i] = fresh ? (LONG)((long long)g_gust.want[i] * g_cfg.bridge_gusts / 100) : 0;
        if (want[i] > 10000) want[i] = 10000;
        if (want[i] < -10000) want[i] = -10000;
        if (i == 1) want[i] = -want[i];
        if (g_gust.obj[i] && (g_gust.dev != dev || g_buf_drop)) {
            drop[i] = g_gust.obj[i];
            g_gust.obj[i] = NULL;
            g_gust.sent[i] = 0;
            g_gust.started[i] = 0;
        }
        if (dev) {
            if (!g_gust.obj[i] && want[i] != 0 && running && g_gust.fails < 6) act[i] = 1;
            else if (g_gust.obj[i] && ((want[i] - g_gust.sent[i] >= 30 || g_gust.sent[i] - want[i] >= 30) ||
                                       (want[i] == 0 && g_gust.sent[i] != 0)))
                act[i] = 2;
        }
        obj[i] = g_gust.obj[i];
        sent[i] = g_gust.sent[i];
    }
    LeaveCriticalSection(&g_cs);
    (void)sent;
    for (i = 0; i < 2; i++) {
        DIEFFECT e;
        DICONSTANTFORCE cf;
        DWORD ax[1];
        LONG dir[1] = {1};
        HRESULT hr;
        if (drop[i]) {
            PFN_Release rel = (PFN_Release)orig_of(drop[i], FX_Release);
            if (rel) rel(drop[i]);
        }
        if (!act[i]) continue;
        ax[0] = axis[i];
        cf.lMagnitude = want[i];
        memset(&e, 0, sizeof e);
        e.dwSize = sizeof e;
        e.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
        e.dwDuration = INFINITE;
        e.dwGain = 10000;
        e.dwTriggerButton = DIEB_NOTRIGGER;
        e.cAxes = 1;
        e.rgdwAxes = ax;
        e.rglDirection = dir;
        e.cbTypeSpecificParams = sizeof cf;
        e.lpvTypeSpecificParams = &cf;
        if (act[i] == 1) {
            PFN_CreateEffect ce = (PFN_CreateEffect)orig_of(dev, DEV_CreateEffect);
            LPDIRECTINPUTEFFECT o = NULL;
            HRESULT h2 = DIERR_GENERIC;
            hr = ce ? ce(dev, &GUID_ConstantForce_, &e, &o, NULL) : DIERR_GENERIC;
            if (SUCCEEDED(hr) && o) {
                PFN_Start st = (PFN_Start)orig_of(o, FX_Start);
                h2 = st ? st(o, 1, 0) : DIERR_GENERIC;
            }
            EnterCriticalSection(&g_cs);
            if (SUCCEEDED(hr) && o) {
                g_gust.obj[i] = o;
                g_gust.dev = dev;
                g_gust.sent[i] = want[i];
                g_gust.started[i] = SUCCEEDED(h2);
                emit(0, "GUST %s force created (start hr=0x%08lx)", i ? "roll" : "pitch", (unsigned long)h2);
            } else {
                g_gust.fails++;
                emit(0, "GUST %s create FAILED hr=0x%08lx", i ? "roll" : "pitch", (unsigned long)hr);
            }
            LeaveCriticalSection(&g_cs);
        } else {
            PFN_SetParameters spf = (PFN_SetParameters)orig_of(obj[i], FX_SetParameters);
            hr = spf ? spf(obj[i], &e, DIEP_TYPESPECIFICPARAMS) : DIERR_GENERIC;
            EnterCriticalSection(&g_cs);
            if (SUCCEEDED(hr)) g_gust.sent[i] = want[i];
            LeaveCriticalSection(&g_cs);
            if (SUCCEEDED(hr) && !g_gust.started[i] && running) {
                PFN_Start st = (PFN_Start)orig_of(obj[i], FX_Start);
                if (st && SUCCEEDED(st(obj[i], 1, 0))) g_gust.started[i] = 1;
            }
        }
    }
}

/* Runs on the heartbeat thread every 50 ms: wing-break residual and the linkage effects.
 * State is read under g_cs; DirectInput is only called with the lock released. */
static void maintain(void)
{
    void *bobj = NULL, *zobj = NULL, *dev = NULL, *drop[3] = {NULL, NULL, NULL};
    DWORD bgain = 0;
    LONG want[3] = {0, 0, 0};
    int act[3] = {0, 0, 0};          /* 1 create, 2 update, 3 start, 4 stop */
    int i, running = 0, restart;
    DIEFFECT base;
    DWORD axes[4];
    LONG dirs[4];
    DICONDITION cond[4];
    Fx *sp;

    EnterCriticalSection(&g_cs);
    maybe_reload();                   /* ini edits apply even while Condor sends nothing */
    /* wing-break residual follows airspeed (Condor's spring stiffness is proportional to it) */
    if (g_brk.active) {
        Fx *bf = fx_find(g_brk.obj);
        LONG k = spring_k_now();
        if (!bf) brk_clear("effect gone", 0);
        else if (GetTickCount() - g_brk.t0 > 120000) brk_clear("2 minutes", 1);
        else if (g_brk.k_break > 0 && k >= 0 && k * 10 < g_brk.k_break) brk_clear("slowed right down", 1);
        else {
            DWORD r = residual_gain();
            if ((r > g_brk.sent ? r - g_brk.sent : g_brk.sent - r) >= 100) {
                bobj = g_brk.obj;
                bgain = r;
                g_brk.sent = r;
                bf->dev_gain = r;
            }
        }
    }
    if (g_brk.zero && fx_find(g_brk.zero)) zobj = g_brk.zero;
    g_brk.zero = NULL;

    /* linkage effects ride on the same axes as Condor's spring */
    sp = latest_named("Spring");
    restart = g_link_restart;
    g_link_restart = 0;
    if (g_link_drop) {
        for (i = 0; i < 3; i++) { drop[i] = g_link[i].obj; g_link[i].obj = NULL; g_link[i].started = 0; }
        g_link_drop = 0;
    }
    if (sp && sp->dev && sp->base.cAxes > 0) {
        LONG k = sp->ts_len >= sizeof(DICONDITION) ? ((DICONDITION *)sp->ts)->lPositiveCoefficient : 0;
        DWORD n = sp->base.cAxes;
        running = sp->running;
        dev = sp->dev;
        base = sp->base;
        memcpy(axes, sp->axes, sizeof axes);
        memcpy(dirs, sp->dirs, sizeof dirs);
        want[0] = g_cfg.link_friction * 100;
        want[1] = g_cfg.link_inertia * 100;
        want[2] = (LONG)((g_cfg.link_damper + g_cfg.link_damper_speed * (k > 0 ? k : 0) / 10000.0) * 100.0);
        for (i = 0; i < 3; i++) {
            if (want[i] > 10000) want[i] = 10000;
            if (g_link[i].obj && g_link[i].dev != dev) {      /* Condor recreated its effects */
                drop[i] = g_link[i].obj;
                g_link[i].obj = NULL;
                g_link[i].started = 0;
            }
            if (!g_link[i].obj) {
                if (want[i] > 0 && running && g_link[i].fails < 3) act[i] = 1;
            } else if ((want[i] > g_link[i].sent ? want[i] - g_link[i].sent : g_link[i].sent - want[i]) >= 100 ||
                       (want[i] == 0 && g_link[i].sent != 0)) {
                act[i] = 2;
            } else if (running && (!g_link[i].started || restart)) {
                act[i] = 3;
            } else if (!running && g_link[i].started) {
                act[i] = 4;
            }
        }
        for (i = 0; i < (int)n && i < 4; i++) {
            memset(&cond[i], 0, sizeof cond[i]);
            cond[i].dwPositiveSaturation = cond[i].dwNegativeSaturation = 10000;
        }
        base.dwSize = sizeof(DIEFFECT);
        base.rgdwAxes = axes;
        base.rglDirection = dirs;
        base.lpEnvelope = NULL;
        base.dwGain = 10000;
        base.dwDuration = INFINITE;
        base.dwTriggerButton = DIEB_NOTRIGGER;
        base.cbTypeSpecificParams = n * sizeof(DICONDITION);
        base.lpvTypeSpecificParams = cond;
    } else {
        for (i = 0; i < 3; i++)
            if (g_link[i].obj) { drop[i] = g_link[i].obj; g_link[i].obj = NULL; g_link[i].started = 0; }
    }
    LeaveCriticalSection(&g_cs);

    if (bobj) set_gain_now(bobj, bgain);
    if (zobj) set_gain_now(zobj, 0);
    bridge_buffet();
    gust_forces();
    EnterCriticalSection(&g_cs);
    g_buf_drop = 0;
    LeaveCriticalSection(&g_cs);
    for (i = 0; i < 3; i++) {
        if (drop[i]) {
            PFN_Release rel = (PFN_Release)orig_of(drop[i], FX_Release);
            if (rel) rel(drop[i]);
        }
    }
    for (i = 0; i < 3; i++) {
        DWORD a;
        HRESULT hr = DI_OK;
        if (!act[i]) continue;
        for (a = 0; a < base.cAxes && a < 4; a++)
            cond[a].lPositiveCoefficient = cond[a].lNegativeCoefficient = want[i];
        if (act[i] == 1) {
            PFN_CreateEffect ce = (PFN_CreateEffect)orig_of(dev, DEV_CreateEffect);
            LPDIRECTINPUTEFFECT o = NULL;
            const GUID *g = i == 0 ? &GUID_Friction_ : i == 1 ? &GUID_Inertia_ : &GUID_Damper_;
            hr = ce ? ce(dev, g, &base, &o, NULL) : DIERR_GENERIC;
            if (SUCCEEDED(hr) && o) {
                PFN_Start st = (PFN_Start)orig_of(o, FX_Start);
                HRESULT h2 = st ? st(o, 1, 0) : DIERR_GENERIC;
                EnterCriticalSection(&g_cs);
                g_link[i].obj = o;
                g_link[i].dev = dev;
                g_link[i].sent = want[i];
                g_link[i].started = SUCCEEDED(h2);
                emit(0, "LINKAGE %s created at %ld (start hr=0x%08lx)", LINK_NAME[i], (long)want[i], (unsigned long)h2);
                LeaveCriticalSection(&g_cs);
            } else {
                EnterCriticalSection(&g_cs);
                g_link[i].fails++;
                emit(0, "LINKAGE %s create FAILED hr=0x%08lx (try %d of 3) - the stick may not support it",
                     LINK_NAME[i], (unsigned long)hr, g_link[i].fails);
                LeaveCriticalSection(&g_cs);
            }
        } else if (act[i] == 2) {
            void *o = g_link[i].obj;
            PFN_SetParameters spf = (PFN_SetParameters)orig_of(o, FX_SetParameters);
            hr = spf ? spf(o, &base, DIEP_TYPESPECIFICPARAMS) : DIERR_GENERIC;
            EnterCriticalSection(&g_cs);
            if (SUCCEEDED(hr)) g_link[i].sent = want[i];
            else emit(0, "LINKAGE %s update FAILED hr=0x%08lx", LINK_NAME[i], (unsigned long)hr);
            LeaveCriticalSection(&g_cs);
        } else if (act[i] == 3 || act[i] == 4) {
            void *o = g_link[i].obj;
            if (act[i] == 3) {
                PFN_Start st = (PFN_Start)orig_of(o, FX_Start);
                hr = st ? st(o, 1, 0) : DIERR_GENERIC;
            } else {
                PFN_Stop sto = (PFN_Stop)orig_of(o, FX_Stop);
                hr = sto ? sto(o) : DIERR_GENERIC;
            }
            EnterCriticalSection(&g_cs);
            g_link[i].started = act[i] == 3 && SUCCEEDED(hr);
            LeaveCriticalSection(&g_cs);
        }
    }
}

static DWORD WINAPI heartbeat(LPVOID arg)
{
    unsigned seq = 0;
    (void)arg;
    for (;;) {
        Fx *s1, *s2, *sp;
        Sleep(50);
        maintain();
        EnterCriticalSection(&g_cs);
        if (g_cfg.udp_port > 0) {
            s1 = latest_named("Stall1");
            s2 = latest_named("Stall2");
            sp = latest_named("Spring");
            emit(2, "STATE seq=%u stall1=%ld stall2=%ld spring_k=%ld broken=%d setp=%u buffet=%lu", seq++,
                 s1 ? (long)s1->gain : -1L, s2 ? (long)s2->gain : -1L,
                 sp && sp->ts_len >= sizeof(DICONDITION) ? (long)((DICONDITION *)sp->ts)->lPositiveCoefficient : -1L,
                 g_brk.active, g_setp_calls, (unsigned long)g_buf.sent);
        }
        LeaveCriticalSection(&g_cs);
    }
    return 0;
}

/* ------------------------------------------------------------------ device hooks */
typedef HRESULT (WINAPI *PFN_SetProperty)(void *, REFGUID, LPCDIPROPHEADER);
typedef HRESULT (WINAPI *PFN_SendFFCmd)(void *, DWORD);
typedef HRESULT (WINAPI *PFN_EnumFile)(void *, const void *, LPDIENUMEFFECTSINFILECALLBACK, LPVOID, DWORD);
typedef HRESULT (WINAPI *PFN_GetDeviceInfo)(void *, void *);

static void name_for_new_effect(int sub, LPCDIEFFECT e, char *out, int cap)
{
    int i;
    if (g_pending_name[0] && g_pending_tid == GetCurrentThreadId()) {
        snprintf(out, cap, "%s", g_pending_name);
        return;
    }
    for (i = 0; i < g_nfilefx; i++) {
        if (g_filefx[i].sub != sub) continue;
        if (IS_PERIODIC(sub) && e && e->lpvTypeSpecificParams && e->cbTypeSpecificParams >= sizeof(DIPERIODIC) &&
            ((const DIPERIODIC *)e->lpvTypeSpecificParams)->dwPeriod != g_filefx[i].period)
            continue;
        snprintf(out, cap, "%s", g_filefx[i].name);
        return;
    }
    snprintf(out, cap, "-");
}

static HRESULT WINAPI Hook_CreateEffect(void *self, REFGUID rguid, LPCDIEFFECT lpeff,
                                        LPDIRECTINPUTEFFECT *ppdeff, LPUNKNOWN outer)
{
    PFN_CreateEffect orig;
    Scaled s;
    int clip = 0, use, sub;
    HRESULT hr;
    const DWORD all = DIEP_TYPESPECIFICPARAMS | DIEP_ENVELOPE;
    EnterCriticalSection(&g_cs);
    maybe_reload();
    orig = (PFN_CreateEffect)orig_of(self, DEV_CreateEffect);
    sub = fx_sub(rguid);
    use = make_scaled(sub, lpeff, all, &s, &clip);   /* periodic: scaled later, through the gain */
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, rguid, use ? &s.e : lpeff, ppdeff, outer);
    EnterCriticalSection(&g_cs);
    if (SUCCEEDED(hr) && ppdeff && *ppdeff) {
        void *fx = *ppdeff;
        Fx *f = fx_add(fx);
        char line[1500];
        int n;
        f->sub = sub;
        f->dev = self;
        name_for_new_effect(sub, lpeff, f->name, sizeof f->name);
        remember(f, lpeff, all | DIEP_GAIN);
        if (lpeff && lpeff->dwSize >= sizeof(DIEFFECT_DX5) && lpeff->dwSize <= sizeof(DIEFFECT)) {
            DWORD i;
            memcpy(&f->base, lpeff, lpeff->dwSize);
            f->base.cAxes = lpeff->cAxes > 4 ? 4 : lpeff->cAxes;
            for (i = 0; i < f->base.cAxes; i++) {
                f->axes[i] = lpeff->rgdwAxes ? lpeff->rgdwAxes[i] : 0;
                f->dirs[i] = lpeff->rglDirection ? lpeff->rglDirection[i] : 0;
            }
        }
        f->dev_gain = lpeff ? lpeff->dwGain : 10000;
        f->dev_mag = (IS_PERIODIC(sub) && f->ts_len >= sizeof(DIPERIODIC)) ? ((DIPERIODIC *)f->ts)->dwMagnitude : 0;
        patch_slot(*(void ***)fx, FX_Release, (void *)Hook_Release);
        patch_slot(*(void ***)fx, FX_GetParameters, (void *)Hook_GetParameters);
        patch_slot(*(void ***)fx, FX_SetParameters, (void *)Hook_SetParameters);
        patch_slot(*(void ***)fx, FX_Start, (void *)Hook_Start);
        patch_slot(*(void ***)fx, FX_Stop, (void *)Hook_Stop);
        n = snprintf(line, sizeof line, "CREATE fx=%d %s %s dev=%p", f->id, f->name, sub_name(sub), self);
        if (lpeff) {
            n += snprintf(line + n, sizeof line - n, " eflags=0x%lx", (unsigned long)lpeff->dwFlags);
            n += fmt_common(line + n, sizeof line - n, lpeff,
                            DIEP_GAIN | DIEP_DURATION | DIEP_AXES | DIEP_DIRECTION | DIEP_ENVELOPE);
            n += fmt_ts(line + n, sizeof line - n, sub, (const BYTE *)lpeff->lpvTypeSpecificParams,
                        lpeff->cbTypeSpecificParams);
        }
        if (use) n += snprintf(line + n, sizeof line - n, " x%d%%%s", pct_for(sub), clip ? " CLIPPED" : "");
        emit(0, "%s", line);
    } else {
        emit(0, "CREATE-FAILED %s hr=0x%08lx", sub_name(sub), (unsigned long)hr);
    }
    LeaveCriticalSection(&g_cs);
    return hr;
}

static HRESULT WINAPI Hook_SetProperty(void *self, REFGUID prop, LPCDIPROPHEADER ph)
{
    PFN_SetProperty orig;
    ULONG_PTR id = (ULONG_PTR)prop;
    DIPROPDWORD d;
    LPCDIPROPHEADER use = ph;
    EnterCriticalSection(&g_cs);
    maybe_reload();
    orig = (PFN_SetProperty)orig_of(self, DEV_SetProperty);
    if ((id == 7 || id == 9) && ph && ph->dwSize >= sizeof(DIPROPDWORD)) {   /* FFGAIN, AUTOCENTER */
        d = *(const DIPROPDWORD *)ph;
        if (id == 7 && g_cfg.master != 100) {
            long long v = (long long)d.dwData * g_cfg.master / 100;
            d.dwData = (DWORD)(v > 10000 ? 10000 : v);
            use = &d.diph;
        }
        emit(0, "%s dev=%p value=%lu%s", id == 7 ? "FFGAIN" : "AUTOCENTER", self,
             (unsigned long)((const DIPROPDWORD *)ph)->dwData, use != ph ? " (master scaled)" : "");
    }
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    return orig(self, prop, use);
}

static HRESULT WINAPI Hook_SendFFCmd(void *self, DWORD cmd)
{
    PFN_SendFFCmd orig;
    EnterCriticalSection(&g_cs);
    orig = (PFN_SendFFCmd)orig_of(self, DEV_SendForceFeedbackCommand);
    emit(0, "FFCMD dev=%p cmd=0x%lx", self, (unsigned long)cmd);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    if (cmd & DISFFC_RESET) drop_assists(self);
    return orig(self, cmd);
}

static LPDIENUMEFFECTSINFILECALLBACK g_user_cb;
static BOOL CALLBACK FileFxCb(LPCDIFILEEFFECT fe, LPVOID ref)
{
    BOOL r;
    EnterCriticalSection(&g_cs);
    if (fe) {
        int sub = fx_sub(&fe->GuidEffect);
        char line[1500];
        int n = snprintf(line, sizeof line, "FILEFX \"%s\" %s", fe->szFriendlyName, sub_name(sub));
        if (fe->lpDiEffect) {
            n += fmt_common(line + n, sizeof line - n, fe->lpDiEffect,
                            DIEP_GAIN | DIEP_DURATION | DIEP_AXES | DIEP_DIRECTION | DIEP_ENVELOPE);
            n += fmt_ts(line + n, sizeof line - n, sub, (const BYTE *)fe->lpDiEffect->lpvTypeSpecificParams,
                        fe->lpDiEffect->cbTypeSpecificParams);
        }
        emit(0, "%s", line);
        if (g_nfilefx < (int)(sizeof g_filefx / sizeof g_filefx[0])) {
            FileFx *x = &g_filefx[g_nfilefx++];
            x->sub = sub;
            x->period = 0;
            if (IS_PERIODIC(sub) && fe->lpDiEffect && fe->lpDiEffect->lpvTypeSpecificParams &&
                fe->lpDiEffect->cbTypeSpecificParams >= sizeof(DIPERIODIC))
                x->period = ((const DIPERIODIC *)fe->lpDiEffect->lpvTypeSpecificParams)->dwPeriod;
            snprintf(x->name, sizeof x->name, "%s", fe->szFriendlyName);
        }
        snprintf(g_pending_name, sizeof g_pending_name, "%s", fe->szFriendlyName);
        g_pending_tid = GetCurrentThreadId();
    }
    LeaveCriticalSection(&g_cs);
    r = g_user_cb ? g_user_cb(fe, ref) : DIENUM_STOP;
    EnterCriticalSection(&g_cs);
    g_pending_name[0] = 0;
    LeaveCriticalSection(&g_cs);
    return r;
}

static HRESULT WINAPI Hook_EnumFile(void *self, const void *file, LPDIENUMEFFECTSINFILECALLBACK cb, LPVOID ref,
                                    DWORD flags)
{
    PFN_EnumFile orig;
    HRESULT hr;
    EnterCriticalSection(&g_cs);
    orig = (PFN_EnumFile)orig_of(self, DEV_EnumEffectsInFile);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    if (!cb) return orig(self, file, cb, ref, flags);
    g_user_cb = cb;
    hr = orig(self, file, FileFxCb, ref, flags);
    g_user_cb = NULL;
    return hr;
}

/* ------------------------------------------------------------------ IDirectInput8 hook */
typedef HRESULT (WINAPI *PFN_CreateDevice)(void *, REFGUID, LPDIRECTINPUTDEVICE8A *, LPUNKNOWN);

static void log_device(void *dev)
{
    PFN_GetDeviceInfo gdi = (PFN_GetDeviceInfo)(*(void ***)dev)[DEV_GetDeviceInfo];
    char name[300] = "?";
    DWORD type = 0;
    GUID ffdrv;
    static const GUID zero;
    memset(&ffdrv, 0, sizeof ffdrv);
    if (g_wide) {
        DIDEVICEINSTANCEW w;
        memset(&w, 0, sizeof w);
        w.dwSize = sizeof w;
        if (SUCCEEDED(gdi(dev, &w))) {
            WideCharToMultiByte(CP_UTF8, 0, w.tszProductName, -1, name, sizeof name, NULL, NULL);
            type = w.dwDevType;
            ffdrv = w.guidFFDriver;
        }
    } else {
        DIDEVICEINSTANCEA a;
        memset(&a, 0, sizeof a);
        a.dwSize = sizeof a;
        if (SUCCEEDED(gdi(dev, &a))) {
            snprintf(name, sizeof name, "%s", a.tszProductName);
            type = a.dwDevType;
            ffdrv = a.guidFFDriver;
        }
    }
    EnterCriticalSection(&g_cs);
    emit(0, "DEVICE dev=%p \"%s\" type=0x%lx ff=%s", dev, name, (unsigned long)type,
         memcmp(&ffdrv, &zero, sizeof ffdrv) ? "yes" : "no");
    LeaveCriticalSection(&g_cs);
}

static HRESULT WINAPI Hook_CreateDevice(void *self, REFGUID rguid, LPDIRECTINPUTDEVICE8A *pp, LPUNKNOWN outer)
{
    PFN_CreateDevice orig;
    HRESULT hr;
    EnterCriticalSection(&g_cs);
    orig = (PFN_CreateDevice)orig_of(self, DI8_CreateDevice);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, rguid, pp, outer);
    if (SUCCEEDED(hr) && pp && *pp) {
        void **vt = *(void ***)*pp;
        EnterCriticalSection(&g_cs);
        patch_slot(vt, DEV_SetProperty, (void *)Hook_SetProperty);
        patch_slot(vt, DEV_CreateEffect, (void *)Hook_CreateEffect);
        patch_slot(vt, DEV_SendForceFeedbackCommand, (void *)Hook_SendFFCmd);
        patch_slot(vt, DEV_EnumEffectsInFile, (void *)Hook_EnumFile);
        LeaveCriticalSection(&g_cs);
        log_device(*pp);
    }
    return hr;
}

/* ------------------------------------------------------------------ exports */
static const GUID IID_DI8W_ = {0xBF798031, 0x483A, 0x4DA2, {0xAA, 0x99, 0x5D, 0x64, 0xED, 0x36, 0x97, 0x00}};

HRESULT WINAPI DirectInput8Create(HINSTANCE h, DWORD ver, REFIID riid, LPVOID *ppv, LPUNKNOWN outer)
{
    HRESULT hr;
    static int first = 1;
    EnterCriticalSection(&g_cs);
    if (first) {
        char exe[MAX_PATH] = "?";
        first = 0;
        read_cfg(0);
        GetModuleFileNameA(NULL, exe, MAX_PATH);
        emit(0, "=== ffb_shim %s loaded in %s pid=%lu", SHIM_VERSION, exe, (unsigned long)GetCurrentProcessId());
        g_ini_check = GetTickCount() - 1000;
        maybe_reload();
        if (!g_ini_mtime.dwLowDateTime && !g_ini_mtime.dwHighDateTime)
            emit(0, "CONFIG ffb_shim.ini not found next to the dll - everything at 100%%");
        CloseHandle(CreateThread(NULL, 0, heartbeat, NULL, 0, NULL));
    }
    if (!load_real()) {
        emit(0, "ERROR could not load the system dinput8.dll");
        LeaveCriticalSection(&g_cs);
        return DIERR_GENERIC;
    }
    LeaveCriticalSection(&g_cs);
    hr = p_DI8Create(h, ver, riid, ppv, outer);
    if (SUCCEEDED(hr) && ppv && *ppv) {
        EnterCriticalSection(&g_cs);
        g_wide = riid && !memcmp(riid, &IID_DI8W_, sizeof(GUID));
        patch_slot(*(void ***)*ppv, DI8_CreateDevice, (void *)Hook_CreateDevice);
        emit(0, "DirectInput8Create ver=0x%lx %s ok", (unsigned long)ver, g_wide ? "W" : "A");
        LeaveCriticalSection(&g_cs);
    }
    return hr;
}

static FARPROC real_proc(const char *name)
{
    FARPROC p = NULL;
    EnterCriticalSection(&g_cs);
    if (load_real()) p = GetProcAddress(g_real, name);
    LeaveCriticalSection(&g_cs);
    return p;
}

HRESULT WINAPI DllCanUnloadNow(void)
{
    FARPROC p = real_proc("DllCanUnloadNow");
    return p ? ((HRESULT (WINAPI *)(void))p)() : S_FALSE;
}

HRESULT WINAPI DllGetClassObject(REFCLSID c, REFIID i, LPVOID *pp)
{
    FARPROC p = real_proc("DllGetClassObject");
    return p ? ((HRESULT (WINAPI *)(REFCLSID, REFIID, LPVOID *))p)(c, i, pp) : CLASS_E_CLASSNOTAVAILABLE;
}

HRESULT WINAPI DllRegisterServer(void)
{
    FARPROC p = real_proc("DllRegisterServer");
    return p ? ((HRESULT (WINAPI *)(void))p)() : E_FAIL;
}

HRESULT WINAPI DllUnregisterServer(void)
{
    FARPROC p = real_proc("DllUnregisterServer");
    return p ? ((HRESULT (WINAPI *)(void))p)() : E_FAIL;
}

LPCDIDATAFORMAT WINAPI GetdfDIJoystick(void)
{
    FARPROC p = real_proc("GetdfDIJoystick");
    return p ? ((LPCDIDATAFORMAT (WINAPI *)(void))p)() : NULL;
}

BOOL WINAPI DllMain(HINSTANCE h, DWORD reason, LPVOID reserved)
{
    (void)reserved;
    if (reason == DLL_PROCESS_ATTACH) {
        char *slash;
        g_self = h;
        DisableThreadLibraryCalls(h);
        InitializeCriticalSection(&g_cs);
        GetModuleFileNameA(h, g_dir, MAX_PATH);
        slash = strrchr(g_dir, '\\');
        if (slash) slash[1] = 0; else g_dir[0] = 0;
    } else if (reason == DLL_PROCESS_DETACH) {
        if (g_log) fflush(g_log);
    }
    return TRUE;
}
