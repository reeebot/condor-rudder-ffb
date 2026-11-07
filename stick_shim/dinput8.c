/*
 * dinput8.dll shim for Condor 3 - log and boost the force feedback Condor sends to the stick.
 *
 * Drop dinput8.dll + ffb_shim.ini next to Condor.exe. Condor loads this file instead of the
 * Windows one; we load the real DirectInput from the system folder and pass every call
 * straight through. Only the force-feedback calls are touched:
 *
 *   - effect strengths (spring stiffness, shake magnitude, ...) are multiplied by the
 *     percentages in ffb_shim.ini (live reload, no restart needed)
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

#define SHIM_VERSION "0.1"

/* ------------------------------------------------------------------ vtable slots */
enum { DI8_CreateDevice = 3 };
enum {
    DEV_SetProperty = 6, DEV_GetDeviceInfo = 15, DEV_CreateEffect = 18,
    DEV_SendForceFeedbackCommand = 22, DEV_EnumEffectsInFile = 27
};
enum { FX_GetParameters = 5, FX_SetParameters = 6, FX_Start = 7, FX_Stop = 8 };

/* ------------------------------------------------------------------ globals */
typedef HRESULT (WINAPI *PFN_DI8Create)(HINSTANCE, DWORD, REFIID, LPVOID *, LPUNKNOWN);

static HMODULE g_self, g_real;
static PFN_DI8Create p_DI8Create;
static CRITICAL_SECTION g_cs;
static char g_dir[MAX_PATH];
static int g_wide;

typedef struct {
    int spring, damper, shake, constant, master;
    int log, log_setp, max_mb, udp_port;
} Cfg;
static Cfg g_cfg = {100, 100, 100, 100, 100, 1, 1, 50, 55291};
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
    if (!g_cfg.log) return;
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
    c.log = GetPrivateProfileIntA("Log", "enabled", 1, ini) != 0;
    c.log_setp = GetPrivateProfileIntA("Log", "log_changes", 1, ini) != 0;
    c.max_mb = clampi(GetPrivateProfileIntA("Log", "max_mb", 50, ini), 1, 2000);
    c.udp_port = clampi(GetPrivateProfileIntA("Bridge", "udp_port", 55291, ini), 0, 65535);
    g_cfg = c;
    if (announce)
        emit(0, "CONFIG spring=%d damper=%d shake=%d constant=%d master=%d log=%d log_changes=%d udp_port=%d",
             c.spring, c.damper, c.shake, c.constant, c.master, c.log, c.log_setp, c.udp_port);
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

typedef struct { DIEFFECT e; BYTE ts[TS_MAX]; DIENVELOPE env; } Scaled;

/* Copy *in to *out with strengths scaled. Returns 1 if anything changed. */
static int make_scaled(int sub, LPCDIEFFECT in, DWORD flags, Scaled *out, int *clip)
{
    int pct = pct_for(sub), changed = 0;
    DWORD sz, i;
    *clip = 0;
    if (!in || pct == 100 || sub < 0) return 0;
    sz = in->dwSize;
    if (sz < sizeof(DIEFFECT_DX5) || sz > sizeof(DIEFFECT)) return 0;
    memset(out, 0, sizeof *out);
    memcpy(&out->e, in, sz);
    if ((flags & DIEP_TYPESPECIFICPARAMS) && in->lpvTypeSpecificParams &&
        in->cbTypeSpecificParams > 0 && in->cbTypeSpecificParams <= TS_MAX) {
        memcpy(out->ts, in->lpvTypeSpecificParams, in->cbTypeSpecificParams);
        if (IS_CONDITION(sub)) {
            DICONDITION *c = (DICONDITION *)out->ts;
            for (i = 0; i < in->cbTypeSpecificParams / sizeof(DICONDITION); i++) {
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
        } else if (IS_PERIODIC(sub) && in->cbTypeSpecificParams >= sizeof(DIPERIODIC)) {
            DIPERIODIC *p = (DIPERIODIC *)out->ts;
            p->dwMagnitude = (DWORD)scl((LONG)p->dwMagnitude, pct, 10000, clip);
            p->lOffset = scl(p->lOffset, pct, 10000, clip);
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
    if ((flags & DIEP_ENVELOPE) && in->lpEnvelope && in->lpEnvelope->dwSize >= sizeof(DIENVELOPE)) {
        int c2 = 0;
        out->env = *in->lpEnvelope;
        out->env.dwAttackLevel = (DWORD)scl((LONG)out->env.dwAttackLevel, pct, 10000, &c2);
        out->env.dwFadeLevel = (DWORD)scl((LONG)out->env.dwFadeLevel, pct, 10000, &c2);
        out->e.lpEnvelope = &out->env;
        changed = 1;
    }
    return changed;
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

static HRESULT WINAPI Hook_SetParameters(void *self, LPCDIEFFECT peff, DWORD flags)
{
    PFN_SetParameters orig;
    Fx *f;
    Scaled s;
    int clip = 0, use;
    HRESULT hr;
    char line[1500];
    int n;
    EnterCriticalSection(&g_cs);
    maybe_reload();
    orig = (PFN_SetParameters)orig_of(self, FX_SetParameters);
    f = fx_find(self);
    use = f ? make_scaled(f->sub, peff, flags, &s, &clip) : 0;
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, use ? &s.e : peff, flags);
    EnterCriticalSection(&g_cs);
    if (f && peff) {
        unsigned sig = sig_of(peff, flags);
        remember(f, peff, flags);
        if (sig != f->sig || FAILED(hr) || (flags & DIEP_START)) {
            f->sig = sig;
            n = snprintf(line, sizeof line, "SETP fx=%d %s %s flags=0x%lx", f->id, f->name,
                         sub_name(f->sub), (unsigned long)flags);
            n += fmt_common(line + n, sizeof line - n, peff, flags);
            if (flags & DIEP_TYPESPECIFICPARAMS)
                n += fmt_ts(line + n, sizeof line - n, f->sub, (const BYTE *)peff->lpvTypeSpecificParams,
                            peff->cbTypeSpecificParams);
            if (use) n += snprintf(line + n, sizeof line - n, " x%d%%%s", pct_for(f->sub), clip ? " CLIPPED" : "");
            if (FAILED(hr)) n += snprintf(line + n, sizeof line - n, " hr=0x%08lx", (unsigned long)hr);
            emit((flags & DIEP_START) || FAILED(hr) ? 0 : 1, "%s", line);
        }
    }
    LeaveCriticalSection(&g_cs);
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
    EnterCriticalSection(&g_cs);
    orig = (PFN_Start)orig_of(self, FX_Start);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self, iter, flags);
    EnterCriticalSection(&g_cs);
    f = fx_find(self);
    emit(0, "START fx=%d %s %s iter=%lu flags=0x%lx hr=0x%08lx", f ? f->id : -1, f ? f->name : "?",
         sub_name(f ? f->sub : -1), (unsigned long)iter, (unsigned long)flags, (unsigned long)hr);
    LeaveCriticalSection(&g_cs);
    return hr;
}

static HRESULT WINAPI Hook_Stop(void *self)
{
    PFN_Stop orig;
    HRESULT hr;
    Fx *f;
    EnterCriticalSection(&g_cs);
    orig = (PFN_Stop)orig_of(self, FX_Stop);
    LeaveCriticalSection(&g_cs);
    if (!orig) return DIERR_GENERIC;
    hr = orig(self);
    EnterCriticalSection(&g_cs);
    f = fx_find(self);
    emit(0, "STOP fx=%d %s %s hr=0x%08lx", f ? f->id : -1, f ? f->name : "?", sub_name(f ? f->sub : -1),
         (unsigned long)hr);
    LeaveCriticalSection(&g_cs);
    return hr;
}

/* ------------------------------------------------------------------ device hooks */
typedef HRESULT (WINAPI *PFN_CreateEffect)(void *, REFGUID, LPCDIEFFECT, LPDIRECTINPUTEFFECT *, LPUNKNOWN);
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
    use = make_scaled(sub, lpeff, all, &s, &clip);
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
        name_for_new_effect(sub, lpeff, f->name, sizeof f->name);
        remember(f, lpeff, all | DIEP_GAIN);
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
    emit(0, "DEVICE dev=%p \"%s\" type=0x%lx ff=%s", dev, name, (unsigned long)type,
         memcmp(&ffdrv, &zero, sizeof ffdrv) ? "yes" : "no");
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
        log_device(*pp);
        LeaveCriticalSection(&g_cs);
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
