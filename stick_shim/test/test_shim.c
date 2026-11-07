/* Drives the shim exactly the way Condor 3 does (captured from a real ffb_shim.log):
 * enum Effects.ffe -> CreateEffect x3 inside the callback -> gains -> Start -> spring
 * stiffness every 50 ms, stall shake through the effect gain. Runs against mock_dinput8.dll
 * installed as the "system" dinput8. Exit code = number of failures. */
#define WIN32_LEAN_AND_MEAN
#define DIRECTINPUT_VERSION 0x0800
#include <winsock2.h>
#include <windows.h>
#include <dinput.h>
#include <stdio.h>
#include <string.h>

typedef HRESULT (WINAPI *PFN_C)(HINSTANCE, DWORD, REFIID, LPVOID *, LPUNKNOWN);
typedef LONG (WINAPI *PFN_M)(int);
typedef LONG (WINAPI *PFN_F)(int, int);
static int fails;
#define CHECK(c, ...) do { if (c) printf("  ok   " __VA_ARGS__); else { printf("  FAIL " __VA_ARGS__); fails++; } printf("\n"); } while (0)

static LPDIRECTINPUTDEVICE8A g_dev;
static LPDIRECTINPUTEFFECT g_fx[8];
static int g_nfx;
static PFN_F mfx;
static BOOL CALLBACK cb(LPCDIFILEEFFECT fe, LPVOID ref)
{
    HRESULT hr = IDirectInputDevice8_CreateEffect(g_dev, &fe->GuidEffect, fe->lpDiEffect, &g_fx[g_nfx], NULL);
    if (SUCCEEDED(hr)) g_nfx++;
    return DIENUM_CONTINUE;
}

static void write_ini(const char *dir, int spring, int shake, int port)
{
    char p[MAX_PATH], b[256];
    FILE *f;
    snprintf(p, sizeof p, "%sffb_shim.ini", dir);
    f = fopen(p, "w");
    snprintf(b, sizeof b, "[Gain]\nspring=%d\nshake=%d\n; a comment\nmaster=100\n[Log]\nenabled=1\n[Bridge]\nudp_port=%d\n",
             spring, shake, port);
    fputs(b, f);
    fclose(f);
}

static HRESULT set_gain(LPDIRECTINPUTEFFECT fx, DWORD gain)
{
    DIEFFECT e;
    memset(&e, 0, sizeof e);
    e.dwSize = sizeof e;
    e.dwGain = gain;
    return IDirectInputEffect_SetParameters(fx, &e, DIEP_GAIN | DIEP_NODOWNLOAD);
}

static HRESULT set_spring(LPDIRECTINPUTEFFECT fx, LONG k)
{
    DIEFFECT e;
    DICONDITION c[2] = {{0, k, k, 10000, 10000, 0}, {0, k, k, 10000, 10000, 0}};
    memset(&e, 0, sizeof e);
    e.dwSize = sizeof e;
    e.cbTypeSpecificParams = sizeof c;
    e.lpvTypeSpecificParams = c;
    return IDirectInputEffect_SetParameters(fx, &e, DIEP_TYPESPECIFICPARAMS | DIEP_NODOWNLOAD);
}

int main(void)
{
    char dir[MAX_PATH], *sl, modpath[MAX_PATH], buf[2048], last_state[256] = "";
    HMODULE shim, mock;
    PFN_C create;
    PFN_M mget;
    LPDIRECTINPUT8A di;
    LPDIRECTINPUT8W diw;
    LPDIRECTINPUTDEVICE8W devw;
    WSADATA w;
    SOCKET s;
    struct sockaddr_in a;
    int udp_lines = 0, states = 0, n, saw_stall_start = 0, i;
    u_long nb = 1;
    DIEFFECT e;
    DIPROPDWORD pd;

    GetModuleFileNameA(NULL, dir, MAX_PATH);
    sl = strrchr(dir, '\\'); sl[1] = 0;
    write_ini(dir, 150, 200, 55299);

    WSAStartup(MAKEWORD(2, 2), &w);
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&a, 0, sizeof a); a.sin_family = AF_INET; a.sin_port = htons(55299); a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bind(s, (struct sockaddr *)&a, sizeof a);
    ioctlsocket(s, FIONBIO, &nb);

    shim = LoadLibraryA("dinput8.dll");
    GetModuleFileNameA(shim, modpath, MAX_PATH);
    CHECK(shim && _strnicmp(modpath, dir, strlen(dir)) == 0, "shim loaded from app folder");
    create = (PFN_C)GetProcAddress(shim, "DirectInput8Create");
    if (!create) return 100;
    CHECK(SUCCEEDED(create(GetModuleHandle(NULL), 0x0800, &IID_IDirectInput8A, (void **)&di, NULL)), "DirectInput8Create A");
    mock = GetModuleHandleA("C:\\windows\\system32\\dinput8.dll");
    mget = mock ? (PFN_M)GetProcAddress(mock, "MockGet") : NULL;
    mfx = mock ? (PFN_F)GetProcAddress(mock, "MockFx") : NULL;
    CHECK(mget && mfx && mock != shim, "real (mock) system dinput8 loaded separately");
    if (!mget || !mfx) return 101;

    /* --- Condor's startup sequence */
    CHECK(SUCCEEDED(IDirectInput8_CreateDevice(di, &GUID_Joystick, &g_dev, NULL)), "CreateDevice");
    CHECK(SUCCEEDED(IDirectInputDevice8_EnumEffectsInFile(g_dev, "Effects.ffe", cb, NULL, 0)) && g_nfx == 3,
          "Effects.ffe -> Spring, Stall1, Stall2 created");
    CHECK(mfx(1, 2) == 10000 && mfx(2, 2) == 7100, "shake magnitudes untouched at creation (%ld, %ld)", mfx(1, 2), mfx(2, 2));
    set_gain(g_fx[0], 10000);
    set_gain(g_fx[1], 0);
    set_gain(g_fx[2], 0);
    for (i = 0; i < 3; i++) IDirectInputEffect_Start(g_fx[i], 1, 0);
    CHECK(mfx(0, 3) == 1 && mfx(1, 3) == 1 && mfx(2, 3) == 1, "all three started");

    /* --- spring: k proportional to airspeed (Condor ~ 104 per kt) */
    set_spring(g_fx[0], 5000);                         /* ~48 kt */
    CHECK(mfx(0, 2) == 7500, "spring 5000 x150%% -> %ld", mfx(0, 2));
    CHECK(mfx(-1, 0) == 3, "no assist spring needed yet (%ld effects)", mfx(-1, 0));
    set_spring(g_fx[0], 9000);                         /* ~87 kt */
    CHECK(mfx(0, 2) == 10000, "spring 9000 x150%% -> main capped at %ld", mfx(0, 2));
    CHECK(mfx(-1, 0) == 4 && mfx(3, 0) == 7, "assist spring created");
    CHECK(mfx(3, 2) == 3500 && mfx(3, 5) == 10000, "assist carries the rest: k %ld sat %ld", mfx(3, 2), mfx(3, 5));
    CHECK(mfx(3, 3) == 1, "assist started because Condor's spring is running");
    set_spring(g_fx[0], 9600);
    CHECK(mfx(3, 2) == 4400, "assist follows: k %ld", mfx(3, 2));
    set_spring(g_fx[0], 4000);
    CHECK(mfx(0, 2) == 6000 && mfx(3, 2) == 0, "slow again: main %ld, assist %ld", mfx(0, 2), mfx(3, 2));

    /* --- stall: Condor ramps the GAIN of both triangles */
    set_gain(g_fx[1], 1500);
    set_gain(g_fx[2], 1500);
    CHECK(mfx(1, 1) == 3000 && mfx(1, 2) == 10000, "Stall1 gain 1500 x200%% -> gain %ld mag %ld", mfx(1, 1), mfx(1, 2));
    CHECK(mfx(2, 1) == 3000 && mfx(2, 2) == 7100, "Stall2 gain 1500 x200%% -> gain %ld mag %ld", mfx(2, 1), mfx(2, 2));
    memset(&e, 0, sizeof e); e.dwSize = sizeof e;
    CHECK(SUCCEEDED(IDirectInputEffect_GetParameters(g_fx[1], &e, DIEP_GAIN)) && e.dwGain == 1500,
          "GetParameters gives Condor its own gain back (%lu)", e.dwGain);

    /* --- VNE flutter: Stall2 alone up to full gain -> spills into magnitude */
    set_gain(g_fx[1], 0);
    set_gain(g_fx[2], 10000);
    CHECK(mfx(2, 1) == 10000 && mfx(2, 2) == 10000, "VNE: Stall2 gain 10000 x200%% -> gain %ld mag %ld (clipped)", mfx(2, 1), mfx(2, 2));
    set_gain(g_fx[2], 6000);
    CHECK(mfx(2, 1) == 10000 && mfx(2, 2) == 8520, "Stall2 gain 6000 x200%% -> gain %ld mag %ld (spill 1.2x)", mfx(2, 1), mfx(2, 2));
    set_gain(g_fx[2], 3000);
    CHECK(mfx(2, 1) == 6000 && mfx(2, 2) == 7100, "Stall2 gain 3000 -> gain %ld, magnitude back to %ld", mfx(2, 1), mfx(2, 2));

    CHECK(SUCCEEDED(IDirectInputEffect_Stop(g_fx[1])) && mget(5) >= 1, "Stop works");
    pd.diph.dwSize = sizeof pd; pd.diph.dwHeaderSize = sizeof pd.diph; pd.diph.dwObj = 0; pd.diph.dwHow = DIPH_DEVICE; pd.dwData = 7000;
    CHECK(SUCCEEDED(IDirectInputDevice8_SetProperty(g_dev, DIPROP_FFGAIN, &pd.diph)) && mget(3) == 7000, "FFGAIN passes through (%ld)", mget(3));

    /* --- live reload: shake back to 100%, spring 100% */
    Sleep(1100);
    write_ini(dir, 100, 100, 55299);
    Sleep(700);
    set_gain(g_fx[1], 1200);
    CHECK(mfx(1, 1) == 1200, "after ini edit shake=100: gain 1200 -> %ld", mfx(1, 1));
    set_spring(g_fx[0], 9000);
    CHECK(mfx(0, 2) == 9000 && mfx(3, 2) == 0, "after ini edit spring=100: main %ld, assist %ld", mfx(0, 2), mfx(3, 2));
    for (n = 0; n < 200; n++) set_spring(g_fx[0], 9000);      /* duplicates must not flood the log */

    /* --- teardown: releasing Condor's spring takes the assist with it */
    Sleep(150);
    IDirectInputEffect_Release(g_fx[0]);
    CHECK(mfx(0, 4) == 1 && mfx(3, 4) == 1, "Release spring -> assist released too");

    CHECK(SUCCEEDED(create(GetModuleHandle(NULL), 0x0800, &IID_IDirectInput8W, (void **)&diw, NULL)), "DirectInput8Create W");
    CHECK(SUCCEEDED(IDirectInput8_CreateDevice(diw, &GUID_Joystick, &devw, NULL)), "CreateDevice W");

    while ((n = recv(s, buf, sizeof buf - 1, 0)) > 0) {
        buf[n] = 0;
        udp_lines++;
        if (strstr(buf, "START fx=1 Stall1")) saw_stall_start = 1;
        if (strstr(buf, " STATE ")) { states++; snprintf(last_state, sizeof last_state, "%s", buf); }
    }
    CHECK(saw_stall_start, "UDP carried 'START fx=1 Stall1'");
    CHECK(states >= 20, "heartbeat STATE lines: %d (last: %s)", states, last_state);
    CHECK(strstr(last_state, "stall1=1200 stall2=3000") != NULL, "STATE reports Condor's own gains");
    printf("%d udp lines, %d failure(s)\n", udp_lines, fails);
    return fails;
}
