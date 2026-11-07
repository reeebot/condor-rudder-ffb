/* Fake "system" dinput8.dll for testing the shim under wine: one FFB stick, effects remember
 * exactly what they were given so the test can check the shim's scaling. */
#define WIN32_LEAN_AND_MEAN
#define DIRECTINPUT_VERSION 0x0800
#include <windows.h>
#include <dinput.h>
#include <string.h>

static LONG g_val[16];   /* 1 spring k+, 2 periodic mag, 3 ffgain, 4 starts, 5 stops, 6 spring sat+, 7 creates, 8 ffcmd */

static int sub_of(const GUID *g)
{
    static const BYTE tail[8] = {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35};
    if (g->Data2 == 0x8E33 && g->Data3 == 0x11D0 && !memcmp(g->Data4, tail, 8)) return (int)(g->Data1 - 0x13541C20);
    return -1;
}

static void WINAPI bad(void) { ExitProcess(99); }
static HRESULT WINAPI qi(void *s, REFIID r, void **pp) { *pp = NULL; return E_NOINTERFACE; }
static ULONG WINAPI addref(void *s) { return 2; }
static ULONG WINAPI release(void *s) { return 1; }
static ULONG WINAPI e_release(void *s) { ((struct { void **vt; int sub; BYTE ts[256]; DWORD ts_len; DWORD gain; int started, released; } *)s)->released++; return 0; }

typedef struct { void **vt; int sub; BYTE ts[256]; DWORD ts_len; DWORD gain; int started, released; } Eff;
static Eff g_eff[16];
static int g_neff;

static void take(Eff *e, LPCDIEFFECT p, DWORD flags)
{
    if (flags & DIEP_GAIN) e->gain = p->dwGain;
    if ((flags & DIEP_TYPESPECIFICPARAMS) && p->lpvTypeSpecificParams && p->cbTypeSpecificParams <= 256) {
        memcpy(e->ts, p->lpvTypeSpecificParams, p->cbTypeSpecificParams);
        e->ts_len = p->cbTypeSpecificParams;
        if (e->sub == 7) { g_val[1] = ((DICONDITION *)e->ts)->lPositiveCoefficient; g_val[6] = ((DICONDITION *)e->ts)->dwPositiveSaturation; }
        if (e->sub >= 2 && e->sub <= 6) g_val[2] = ((DIPERIODIC *)e->ts)->dwMagnitude;
    }
}
static HRESULT WINAPI e_getp(void *s, LPDIEFFECT p, DWORD flags)
{
    Eff *e = s;
    if ((flags & DIEP_TYPESPECIFICPARAMS) && p->lpvTypeSpecificParams) {
        if (p->cbTypeSpecificParams < e->ts_len) return DIERR_MOREDATA;
        memcpy(p->lpvTypeSpecificParams, e->ts, e->ts_len);
        p->cbTypeSpecificParams = e->ts_len;
    }
    return DI_OK;
}
static HRESULT WINAPI e_setp(void *s, LPCDIEFFECT p, DWORD flags) { take(s, p, flags); return DI_OK; }
static HRESULT WINAPI e_start(void *s, DWORD it, DWORD fl) { g_val[4]++; ((Eff *)s)->started++; return DI_OK; }
static HRESULT WINAPI e_stop(void *s) { g_val[5]++; return DI_OK; }
static void *EFF_VT[13] = {qi, addref, e_release, bad, bad, e_getp, e_setp, e_start, e_stop, bad, bad, bad, bad};

static HRESULT WINAPI d_setprop(void *s, REFGUID g, LPCDIPROPHEADER ph)
{
    if ((ULONG_PTR)g == 7) g_val[3] = ((const DIPROPDWORD *)ph)->dwData;
    return DI_OK;
}
static HRESULT WINAPI d_getinfo(void *s, void *inst)
{
    DWORD sz = *(DWORD *)inst;
    if (sz == sizeof(DIDEVICEINSTANCEW)) {
        DIDEVICEINSTANCEW *w = inst;
        memset(w, 0, sz); w->dwSize = sz; w->dwDevType = 0x10014;
        lstrcpyW(w->tszProductName, L"Mock CLS-E Joystick");
        w->guidFFDriver.Data1 = 1;
    } else if (sz == sizeof(DIDEVICEINSTANCEA)) {
        DIDEVICEINSTANCEA *a = inst;
        memset(a, 0, sz); a->dwSize = sz; a->dwDevType = 0x10014;
        lstrcpyA(a->tszProductName, "Mock CLS-E Joystick");
        a->guidFFDriver.Data1 = 1;
    } else return DIERR_INVALIDPARAM;
    return DI_OK;
}
static HRESULT WINAPI d_create(void *s, REFGUID g, LPCDIEFFECT p, LPDIRECTINPUTEFFECT *pp, LPUNKNOWN o)
{
    Eff *e = &g_eff[g_neff++];
    e->vt = EFF_VT;
    e->sub = sub_of(g);
    take(e, p, DIEP_TYPESPECIFICPARAMS | DIEP_GAIN);
    g_val[7]++;
    *pp = (LPDIRECTINPUTEFFECT)e;
    return DI_OK;
}
static HRESULT WINAPI d_ffcmd(void *s, DWORD c) { g_val[8] = (LONG)c; return DI_OK; }
static HRESULT WINAPI d_enumfile(void *s, LPCSTR f, LPDIENUMEFFECTSINFILECALLBACK cb, LPVOID ref, DWORD fl)
{
    static DICONDITION spring[2] = {{0, 10000, 10000, 0, 0, 0}, {0, 10000, 10000, 0, 0, 0}};
    static DIPERIODIC tri = {10000, 0, 0, 158900}, tri2 = {7100, 0, 0, 205300};
    static DWORD axes[2] = {0, 4};
    static LONG dir[2] = {0, 0};
    static DIEFFECT es, et;
    DIFILEEFFECT fe;
    memset(&es, 0, sizeof es); es.dwSize = sizeof es; es.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
    es.dwDuration = INFINITE; es.dwGain = 10000; es.cAxes = 2; es.rgdwAxes = axes; es.rglDirection = dir;
    es.cbTypeSpecificParams = sizeof spring; es.lpvTypeSpecificParams = spring;
    et = es; et.cAxes = 1; et.cbTypeSpecificParams = sizeof tri; et.lpvTypeSpecificParams = &tri;
    memset(&fe, 0, sizeof fe); fe.dwSize = sizeof fe;
    fe.GuidEffect = GUID_Spring; fe.lpDiEffect = &es; lstrcpyA(fe.szFriendlyName, "Spring");
    if (cb(&fe, ref) != DIENUM_CONTINUE) return DI_OK;
    fe.GuidEffect = GUID_Triangle; fe.lpDiEffect = &et; lstrcpyA(fe.szFriendlyName, "Stall1");
    if (cb(&fe, ref) != DIENUM_CONTINUE) return DI_OK;
    et.lpvTypeSpecificParams = &tri2;
    fe.lpDiEffect = &et; lstrcpyA(fe.szFriendlyName, "Stall2");
    cb(&fe, ref);
    return DI_OK;
}
static void *DEV_VT[32] = {qi, addref, release, bad, bad, bad, d_setprop, bad, bad, bad, bad, bad, bad, bad, bad,
                           d_getinfo, bad, bad, d_create, bad, bad, bad, d_ffcmd, bad, bad, bad, bad, d_enumfile,
                           bad, bad, bad, bad};
static struct { void **vt; } g_dev = {DEV_VT};

static HRESULT WINAPI di_create(void *s, REFGUID g, void **pp, LPUNKNOWN o) { *pp = &g_dev; return DI_OK; }
static void *DI_VT_A[11] = {qi, addref, release, di_create, bad, bad, bad, bad, bad, bad, bad};
static void *DI_VT_W[11] = {qi, addref, release, di_create, bad, bad, bad, bad, bad, bad, bad};
static struct { void **vt; } g_di_a = {DI_VT_A}, g_di_w = {DI_VT_W};

HRESULT WINAPI DirectInput8Create(HINSTANCE h, DWORD v, REFIID r, LPVOID *pp, LPUNKNOWN o)
{
    *pp = (r->Data1 == 0xBF798031) ? (void *)&g_di_w : (void *)&g_di_a;
    return DI_OK;
}
LONG WINAPI MockGet(int i) { return (i >= 0 && i < 16) ? g_val[i] : -1; }
/* what: 0 sub, 1 gain, 2 magnitude (periodic) or k+ axis 0 (condition), 3 started, 4 released, 5 sat+ */
LONG WINAPI MockFx(int idx, int what)
{
    Eff *e;
    if (idx < 0) return g_neff;
    if (idx >= g_neff) return -999;
    e = &g_eff[idx];
    switch (what) {
    case 0: return e->sub;
    case 1: return (LONG)e->gain;
    case 2: return (e->sub >= 2 && e->sub <= 6) ? (LONG)((DIPERIODIC *)e->ts)->dwMagnitude : ((DICONDITION *)e->ts)->lPositiveCoefficient;
    case 3: return e->started;
    case 4: return e->released;
    case 5: return (LONG)((DICONDITION *)e->ts)->dwPositiveSaturation;
    }
    return -998;
}
