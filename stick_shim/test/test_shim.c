/* Drives the shim the way Condor does (enum Effects.ffe -> CreateEffect -> SetParameters ...)
 * against mock_dinput8.dll installed as the "system" dinput8. Exit code = number of failures. */
#define WIN32_LEAN_AND_MEAN
#define DIRECTINPUT_VERSION 0x0800
#include <winsock2.h>
#include <windows.h>
#include <dinput.h>
#include <stdio.h>
#include <string.h>

typedef HRESULT (WINAPI *PFN_C)(HINSTANCE, DWORD, REFIID, LPVOID *, LPUNKNOWN);
typedef LONG (WINAPI *PFN_M)(int);
static int fails;
#define CHECK(c, ...) do { if (c) printf("  ok   " __VA_ARGS__); else { printf("  FAIL " __VA_ARGS__); fails++; } printf("\n"); } while (0)

static LPDIRECTINPUTDEVICE8A g_dev;
static LPDIRECTINPUTEFFECT g_fx[8];
static int g_nfx;
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
    snprintf(b, sizeof b, "[Gain]\nspring=%d\nshake=%d\n; a comment\nmaster=100\n[Log]\nenabled=1\n[Bridge]\nudp_port=%d\n", spring, shake, port);
    fputs(b, f);
    fclose(f);
}

int main(void)
{
    char dir[MAX_PATH], *sl, modpath[MAX_PATH], buf[2048];
    HMODULE shim, mock;
    PFN_C create;
    PFN_M mget;
    LPDIRECTINPUT8A di;
    LPDIRECTINPUT8W diw;
    LPDIRECTINPUTDEVICE8W devw;
    WSADATA w;
    SOCKET s;
    struct sockaddr_in a;
    int udp_lines = 0, n, saw_stall_start = 0;
    u_long nb = 1;
    DWORD flags = DIEP_TYPESPECIFICPARAMS;
    DICONDITION c[2] = {{0, 8000, 8000, 6000, 6000, 0}, {0, 8000, 8000, 6000, 6000, 0}};
    DICONDITION back[2];
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
    printf("loaded %s\n", modpath);
    CHECK(shim && _strnicmp(modpath, dir, strlen(dir)) == 0, "shim loaded from app folder");
    create = (PFN_C)GetProcAddress(shim, "DirectInput8Create");
    CHECK(create != NULL, "DirectInput8Create exported");
    if (!create) return 100;

    CHECK(SUCCEEDED(create(GetModuleHandle(NULL), 0x0800, &IID_IDirectInput8A, (void **)&di, NULL)), "DirectInput8Create A");
    mock = GetModuleHandleA("C:\\windows\\system32\\dinput8.dll");
    mget = mock ? (PFN_M)GetProcAddress(mock, "MockGet") : NULL;
    CHECK(mget != NULL && mock != shim, "real (mock) system dinput8 loaded separately");
    if (!mget) return 101;

    CHECK(SUCCEEDED(IDirectInput8_CreateDevice(di, &GUID_Joystick, &g_dev, NULL)), "CreateDevice");
    CHECK(SUCCEEDED(IDirectInputDevice8_EnumEffectsInFile(g_dev, "Effects.ffe", cb, NULL, 0)) && g_nfx == 2, "EnumEffectsInFile + CreateEffect x2");
    CHECK(mget(1) == 4500, "Spring created at 150%%: k 3000 -> %ld", mget(1));
    CHECK(mget(6) == 10000, "Spring saturation stays 10000: %ld", mget(6));
    CHECK(mget(2) == 10000, "Stall1 created at 200%% clipped: mag 6000 -> %ld", mget(2));

    memset(&e, 0, sizeof e); e.dwSize = sizeof e; e.cbTypeSpecificParams = sizeof c; e.lpvTypeSpecificParams = c;
    CHECK(SUCCEEDED(IDirectInputEffect_SetParameters(g_fx[0], &e, flags)), "SetParameters spring");
    CHECK(mget(1) == 10000, "spring k 8000 x150%% clipped -> %ld", mget(1));
    CHECK(mget(6) == 9000, "spring saturation 6000 x150%% -> %ld", mget(6));
    CHECK(c[0].lPositiveCoefficient == 8000, "caller's struct untouched (%ld)", c[0].lPositiveCoefficient);
    memset(back, 0, sizeof back);
    e.cbTypeSpecificParams = sizeof back; e.lpvTypeSpecificParams = back;
    CHECK(SUCCEEDED(IDirectInputEffect_GetParameters(g_fx[0], &e, flags)) && back[0].lPositiveCoefficient == 8000,
          "GetParameters returns Condor's own value (%ld), no compounding", back[0].lPositiveCoefficient);

    CHECK(SUCCEEDED(IDirectInputEffect_Start(g_fx[1], 1, 0)) && mget(4) == 1, "Start stall effect");
    CHECK(SUCCEEDED(IDirectInputEffect_Stop(g_fx[1])) && mget(5) == 1, "Stop stall effect");

    pd.diph.dwSize = sizeof pd; pd.diph.dwHeaderSize = sizeof pd.diph; pd.diph.dwObj = 0; pd.diph.dwHow = DIPH_DEVICE; pd.dwData = 7000;
    CHECK(SUCCEEDED(IDirectInputDevice8_SetProperty(g_dev, DIPROP_FFGAIN, &pd.diph)) && mget(3) == 7000, "FFGAIN passes through at master=100 (%ld)", mget(3));
    CHECK(SUCCEEDED(IDirectInputDevice8_SendForceFeedbackCommand(g_dev, DISFFC_RESET)) && mget(8) == DISFFC_RESET, "SendForceFeedbackCommand");

    /* live reload: back to 100% */
    Sleep(1100);
    write_ini(dir, 100, 100, 55299);
    Sleep(700);
    c[0].lPositiveCoefficient = 5000;
    e.cbTypeSpecificParams = sizeof c; e.lpvTypeSpecificParams = c;
    IDirectInputEffect_SetParameters(g_fx[0], &e, flags);
    CHECK(mget(1) == 5000, "after ini edit spring=100: k 5000 -> %ld", mget(1));

    /* 200 identical SetParameters: should not flood the log */
    for (n = 0; n < 200; n++) IDirectInputEffect_SetParameters(g_fx[0], &e, flags);

    /* wide interface + device name */
    CHECK(SUCCEEDED(create(GetModuleHandle(NULL), 0x0800, &IID_IDirectInput8W, (void **)&diw, NULL)), "DirectInput8Create W");
    CHECK(SUCCEEDED(IDirectInput8_CreateDevice(diw, &GUID_Joystick, &devw, NULL)), "CreateDevice W");

    while ((n = recv(s, buf, sizeof buf - 1, 0)) > 0) {
        buf[n] = 0;
        udp_lines++;
        if (strstr(buf, "START fx=1 Stall1")) saw_stall_start = 1;
    }
    CHECK(udp_lines >= 8 && udp_lines < 40, "UDP lines to bridge: %d", udp_lines);
    CHECK(saw_stall_start, "UDP carried 'START fx=1 Stall1'");
    printf("%d failure(s)\n", fails);
    return fails;
}
