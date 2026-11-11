#!/usr/bin/env python3
"""
condor_brunner_ffb.py
=====================
Condor Soaring Simulator (2 / 3)  --UDP-->  this bridge  --UDP-->  Brunner CLS2Sim
                                                                   -> CLS-E MkII rudder

Python 3.8+, standard library only. Runs on the Windows box with Condor + CLS2Sim.

WHAT IT DOES
------------
Uses CLS2Sim Remote Control protocol v2 (NOT "Low level control"), so CLS2Sim keeps
running its own spring/damping loop from your profile while Condor reads the pedals
directly. The bridge only overrides four things on the rudder:

  1. Move back           (0x40)  OFF when slow/stopped: pedals limp, stay where you put them
     Force scale factor  (0x20)  airflow centring stiffness, progressive curve in IAS
  2. Trim position       (0x90)  yaw feel: force builds with sideslip (yaw string)
  3. Fast vibration      (0xC1/2) ground-roll rumble, scaled by speed and surface roughness
  4. Slow vibration      (0xC3/4) stall flutter (IAS vs g-scaled onset), optional turbulence

When Condor stops sending (menu, pause, crash) everything ramps back to neutral.
On Ctrl+C it zeroes trim/vibration and releases all overrides back to CLS2Sim.

SETUP
-----
Condor  ->  <Condor install dir>\\UDP.ini
    [General]
    Enabled=1
    [Connection]
    Host=127.0.0.1
    Port=55278
    [Misc]
    SendIntervalMs=20
    ExtendedData=1        ; needed for wheelheight / surfaceroughness / turbulence
    ExtendedData1=0
    LogToFile=0
  You do NOT need a virtual COM port for this. (Condor's COM/NMEA output is for
  XCSoar-type nav apps and doesn't carry the yaw string.) If something else already
  listens on 55278, run this bridge there and relay with --forward 127.0.0.1:55279.

CLS2Sim  ->  File > Settings
  * Remote control: Enabled.  UDP port 15090.
  * "Low level control" / "External control": OFF (it disables everything else).
  * "Message Identifier": OFF (or set cls2sim_msg_id = true in settings.ini).
  * Simulation: "USB Joystick Emulation". Sim auto-connect OFF, hardware auto-connect ON.
    CLS2Sim will never "connect" to Condor; it only needs to be connected to the pedals.
  * Profile: a normal rudder force profile with "move back" enabled. The bridge scales
    it with airspeed and switches move-back off when slow; it does not replace it.
    RECOMMENDED: divide the 9 rudder force-profile values in CLS2Sim by 3 and set
    profile_divider = 3 in settings.ini. The bridge multiplies everything back up, so the
    feel in flight is unchanged, but CLS2Sim's 20% minimum load (and the limp friction)
    shrinks to ~7%, so the load fades smoothly toward zero with no drop-off at low speed.

Condor  ->  Setup > Controls: rudder mapped to the pedals' own USB device (no vJoy
  needed on the tested setup). Condor's force feedback only drives the stick, so leave
  it on. If the rudder ever goes dead in Condor while CLS2Sim is connected, install
  vJoy from <CLS2Sim dir>\\Virtual joystick driver and map the vJoy axis instead
  (and if stick FFB then dies, untick "Enable Effects" for vJoy in vJoyConf).
  The Brunner stick runs in DirectX mode on its own USB cable, outside CLS2Sim; this
  bridge only touches the rudder axis (axis_mask 0x4).

RUNNING
-------
  python condor_brunner_ffb.py                # the real thing (or double-click run_ffb.bat)
  python condor_brunner_ffb.py --selftest     # pedals only, no Condor: checks link + trim sign
  python condor_brunner_ffb.py --dump         # Condor only: prints the live UDP fields
  python condor_brunner_ffb.py --airbrake-test  # beeps instructions, finds the airbrake field
  python condor_brunner_ffb.py --record       # records all Condor data to CSV until Ctrl+C
  python condor_brunner_ffb.py --lever-setup  # finds your airbrake lever, saves it to settings
  python condor_brunner_ffb.py --dry-run      # Condor in, computed forces printed, nothing sent

TUNING -> settings.ini (next to this script)
  All tuning lives in settings.ini, with a comment on every setting. It's created with
  defaults on first run. Edit and save it WHILE FLYING: changes apply within ~1 s
  (except [Network] and keys marked "restart"). Delete it to get fresh defaults.
  --settings PATH uses another file; --write-settings PATH writes a fresh default one.

YAW FEEL (float_mode) and SIGN CALIBRATION (float_sign)
  float_mode "stream" (default): the rudder floats with the airflow at the tail, like a real
    free rudder. Wind from the left (yaw string blown right) -> the rudder trails right ->
    right pedal drifts forward. In a slip you hold, the pushed pedal gets a bit lighter.
  float_mode "resist": the opposite shift - the pushed pedal pushes back harder the more
    you yaw. Easier to feel, but backwards for a gust or crosswind you didn't cause.
  float_sign is calibration only. Verified on Rebot's CLS-E MkII + Condor 3, 2026-11-05:
    CLS2Sim trim +0.25 -> right pedal forward; left-pedal slip -> yawstringangle negative
    => float_sign = 1. Don't change it unless the hardware or Condor changes.

STALL FLUTTER
  Condor's UDP has no stall warning or AoA, so buffet is triggered from IAS:
  onset = buffet_onset_kt * sqrt(g). Calibrate: fly a straight wings-level stall, note
  the IAS in this window when the stick starts to flutter, put it in buffet_onset_kt.
  Raise it a bit when flying with water ballast.

Protocol reference: https://cls2sim.brunner-innovation.swiss/protocolv2.htm
"""

import argparse
import configparser
import math
import os
import socket
import struct
import sys
import time

# ----------------------------------------------------------------------------
# Defaults. Users tune these in settings.ini (see SETTINGS_LAYOUT below).
# ----------------------------------------------------------------------------
DEFAULTS = {
    # --- networking ---
    "condor_listen_host": "0.0.0.0",
    "condor_listen_port": 55278,
    "forward_to": [],                 # e.g. ["127.0.0.1:55279"] relay raw Condor packets
    "cls2sim_host": "127.0.0.1",
    "cls2sim_port": 15090,
    "cls2sim_msg_id": False,          # True only if "Message Identifier" is ticked in CLS2Sim
    "cls2sim_timeout_s": 0.25,
    "axis_mask": 0x4,                 # Rudder (pilot). 0x4 also covers a copilot rudder.
    "shim_udp_port": 55291,           # stick shim (dinput8.dll in Condor's folder) feed. 0 = off
    "shim_cmd_port": 55292,           # where the bridge sends stick buffet commands to the shim

    # --- loop ---
    "tick_hz": 30,
    "stale_after_s": 0.5,             # no new Condor time for this long -> neutral
    "resend_all_every_s": 30.0,       # re-assert everything (survives Profile Manager "Apply")
    "watchdog_ms": 0,                 # 0=off. >0: CLS2Sim STOPS the drives if the bridge dies

    # --- airflow load vs airspeed. No mechanical spring: limp at rest. ---
    # All speeds in KNOTS (IAS). Below airflow_starts_kt: CLS2Sim "move back" OFF -> pedals
    # limp, stay where you put them. Above: centring from airflow,
    # stiffness = sf_at_vref * (IAS / v_ref_kt) ^ sf_exponent (% of profile, 20..999).
    # Above sf_knee_kt it keeps rising more gently: load_knee * (IAS / knee) ^ sf_exponent_high.
    # Defaults: <5 kt limp | 5-11 kt 20% (CLS2Sim minimum) | 20: 40% | 30: 65% | 40: 92% |
    # 50: 120% | 60: 149% | 70: 180% (3/4 of max) | 90: 201% | 110: 220% | 133+: 240%
    "profile_divider": 1.0,           # set 3 after dividing CLS2Sim rudder profile forces by 3
    "airflow_starts_kt": 3.0,
    "limp_hysteresis_kt": 1.0,        # goes limp again below airflow_starts_kt - this
    "sf_at_vref": 120,                # at v_ref_kt
    "v_ref_kt": 50.0,
    "sf_exponent": 1.2,               # 2.0 = pure dynamic pressure (steep). Lower = more
                                      # feel at low speed relative to high speed.
    "sf_knee_kt": 70.0,               # above this the load keeps rising, but more gently
    "sf_exponent_high": 0.45,         # curve shape above the knee
    "sf_max": 240,

    # --- yaw feel from sideslip (trim position, -1..1) ---
    "float_mode": "stream",           # "stream" = rudder floats with the airflow (real), "resist" = opposite
    "float_gain_per_rad": 1.0,        # trim offset per rad of yaw string. 15 deg -> ~0.26
    "float_sign": 1,                  # CALIBRATION, verified for this setup - leave at 1
    "float_max": 0.5,
    "float_fade_start_kt": 10.0,      # yaw string is noise below this
    "float_fade_full_kt": 25.0,
    "float_rate_per_s": 1.5,          # max trim slew

    # --- ground roll rumble (fast vibration channel). Needs ExtendedData=1 ---
    "ground_vib_enable": True,
    "ground_wheel_height_m": 0.2,     # wheelheight below this = rolling on the ground
    "ground_vib_min_kt": 2.0,
    "ground_vib_full_kt": 45.0,
    "ground_vib_force_max": 250,      # internal units at full speed, nominal surface
                                      # (roughness scales it 0.3x..1.5x, then vib_force_cap).
                                      # Brunner: start ~50, step up by 100.
    "ground_vib_interval_slow_ms": 70,
    "ground_vib_interval_fast_ms": 30,
    "ground_roughness_gain": 1.0,     # force *= clamp(0.5 + gain*roughness, 0.3, 1.5)

    # --- linkage: the cables still move the rudder's mass, even when stopped ---
    "limp_load_pct": 20.0,            # how heavy the limp pedals are at rest (% of original profile)
    "damping_rest": 0.02,             # CLS2Sim friction-by-velocity factor when stopped
    "damping_per_100kt": 0.06,        # extra damping per 100 kt (air on the rudder)
    "damping_limit": 15.0,            # cap on the damping force (N or Nm)

    # --- airborne slow vibration channel: stall flutter (+ optional turbulence) ---
    "stall_source": "auto",           # auto: Condor's own stall shake via the stick shim, else IAS
    "condor_stall_full_gain": 2500,   # Condor shake level (0-10000) = full pedal flutter
    "condor_stall_curve": 1.0,        # 1 = follow Condor's shake linearly
    "buffet_onset_kt": 36.0,          # 1 g flutter onset IAS. 0 = off. Calibrate per glider.
    "buffet_band_kt": 8.0,            # flutter builds to full this far below onset
    "buffet_curve": 1.6,              # ramp shape: 1 = linear, 2 = gentle start then builds
    "buffet_g_scaling": True,         # onset *= sqrt(g) (Condor 'gforce', ExtendedData=1)
    "buffet_force_max": 260,
    "buffet_interval_ms": 50,
    "buffet_buzz_force_max": 140,     # extra fast buzz layered on top near a full stall
    "buffet_buzz_interval_ms": 25,
    "turb_vib_force_per_unit": 0.0,   # force per unit of Condor turbulencestrength (0=off)
    "turb_vib_interval_ms": 90,

    # --- airbrake buffet: Condor doesn't send airbrake position, so read the lever ---
    "airbrake_enable": True,
    "airbrake_device": "",            # VID_xxxx&PID_xxxx - filled in by lever_setup.bat
    "airbrake_axis": "",              # X Y Z R U V       - filled in by lever_setup.bat
    "airbrake_closed_raw": 0,
    "airbrake_open_raw": 65535,
    "airbrake_deadzone": 0.05,
    "airbrake_vib_force_max": 240,    # at full airbrakes and airbrake_vib_full_kt
    "airbrake_vib_full_kt": 110.0,
    "airbrake_vib_min_kt": 15.0,
    "airbrake_vib_low_frac": 0.3,     # share of full buffet you still feel at low speed
    "airbrake_vib_curve": 0.4,        # <1 = builds early (60 kt ~ 80%), keeps growing to full
    "airbrake_vib_interval_ms": 60,

    # --- high-speed buffet: buzz building toward VNE ---
    "hs_buffet_onset_kt": 115.0,      # 0 = off
    "hs_buffet_full_kt": 145.0,       # ~VNE of a typical 15 m / standard class glider
    "hs_buffet_force_max": 140,
    "hs_buffet_curve": 1.5,           # >1 = creeps in, then builds hard near VNE
    "hs_buffet_interval_ms": 28,
    "condor_vne_force_max": 260,      # Condor's own VNE flutter (stick shim), on the pedals. 0 = off
    "condor_vne_interval_ms": 50,

    # --- stick buffet through the stick shim (Condor doesn't shake the stick for these) ---
    "stick_airbrake_buffet": 30,      # % of the stick's max force at full brakes, full speed
    "stick_hs_buffet": 25,            # % at the high-speed buffet's full speed
    "stick_buffet_period_ms": 70,
    # --- stick gust forces: bumps from load-factor and roll-rate changes (not stiffer) ---
    "stick_gust_pitch_per_g": 0.0,    # OFF: closes a loop through your own stick input (see below)
    "stick_gust_roll_per_dps": 0.0,   # OFF, same reason
    "stick_gust_max": 15.0,           # cap, % of max force
    "stick_gust_pitch_deadband_g": 0.08,  # load-factor wiggles smaller than this are ignored
    "stick_gust_roll_deadband_dps": 3.0,  # roll-rate wiggles smaller than this are ignored
    "stick_gust_pitch_sign": -1,      # -1 on Rebot's CLS-E (DirectX mode pushes the other way)
    "stick_gust_roll_sign": 1,        # flip to -1 if rolling drags the stick further into the roll
    "stick_gust_highpass_s": 0.7,     # slower changes than this are ignored (turns, trim)
    "stick_gust_smooth_s": 0.05,      # smooths Condor's data steps

    # --- structural failure: Condor's stick flutter stops dead when a wing breaks ---
    "failure_detect": True,
    "failure_rollrate_dps": 90.0,     # sustained roll rate above this at speed = wing gone
    "failure_hold_s": 0.4,
    "failure_min_kt": 60.0,
    "failure_reset_alt_jump_m": 50.0, # altitude teleport = new flight -> reset
    "failure_residual": 0.4,          # share of the flutter that keeps going after the break

    "vib_force_cap": 400,             # hard cap on any vibration force sent

    # --- filters ---
    "ias_tau_s": 0.15,
    "yaw_tau_s": 0.12,
}

# ----------------------------------------------------------------------------
# settings.ini: layout, comments, read/write, live reload
# ----------------------------------------------------------------------------
SETTINGS_HEADER = """\
; ============================================================================
;  Condor -> Brunner CLS-E rudder force feedback - settings
; ============================================================================
;  Edit and SAVE while flying: changes apply within about 1 second.
;  Exceptions: everything in [Network], plus keys marked (restart).
;  Lines starting with ; are comments. You can also put ; comments after a value.
;  Delete this file to get fresh defaults on the next start.
;
;  Vibration forces are in Brunner internal units. Go up in steps of ~50-100.
; ============================================================================
"""

SETTINGS_LAYOUT = [
    ("Airflow",
     "Airflow load on the rudder linkage. There is NO spring: stopped, the pedals are limp\n"
     "and stay where you put them. From airflow_starts_kt, airflow centres them:\n"
     "stiffness = sf_at_vref * (IAS / v_ref_kt) ^ sf_exponent  (% of your ORIGINAL profile)\n"
     "Above sf_knee_kt it keeps building, more gently, until sf_max:\n"
     "stiffness = load_at_knee * (IAS / sf_knee_kt) ^ sf_exponent_high\n"
     "Curve: 5 kt: 8% | 10: 17% | 20: 40% | 30: 65% | 40: 92% | 50: 120% | 60: 149% |\n"
     "70: 180% (3/4 of max) | 80: 191% | 90: 201% | 100: 211% | 110: 220% | 120: 229% |\n"
     "133+: 240%. More than 100% is fine - it's % of your CLS2Sim profile (max 999).\n"
     "CLS2Sim can't go below a 20% load. With profile_divider = 1 that floor holds the load\n"
     "at 20% from 3 to 11 kt, then it drops to limp. With profile_divider = 3 the floor is\n"
     "~7%: the curve fades smoothly down to 4.5 kt and the last step to limp is tiny.",
     [("profile_divider", "Divide the 9 force values of your CLS2Sim rudder profile by this,\n"
                          "then set it here; the bridge scales back up so the feel is the same.\n"
                          "3 is the sweet spot (max ~3.5 before sf_max x divider hits 999).\n"
                          "Leave at 1 if you haven't changed the CLS2Sim profile!"),
      ("airflow_starts_kt", "Centring and resistance start above this IAS (kt)."),
      ("limp_hysteresis_kt", "Goes limp again below airflow_starts_kt minus this (kt).\n"
                             "Stops it flickering on/off right at the threshold."),
      ("sf_at_vref", "Stiffness at v_ref_kt (%)."),
      ("v_ref_kt", "Reference airspeed for sf_at_vref (kt)."),
      ("sf_exponent", "Curve shape. 2.0 = steep (soft slow, heavy fast). 1.0 = straight line.\n"
                      "Too stiff at medium-high speed: lower this or sf_at_vref.\n"
                      "Lower it if slow speeds feel too soft compared to fast."),
      ("sf_knee_kt", "Above this IAS the stiffness keeps rising, but more gently (kt)."),
      ("sf_exponent_high", "Curve shape above the knee. 0 = flat (no more increase),\n"
                           "1 = keeps rising in a straight line. Raise it for more at high speed."),
      ("sf_max", "Hard ceiling on stiffness (%). The curve reaches it at ~133 kt by default.")]),
    ("Linkage",
     "The cable linkage still has to move the rudder's mass and fight its friction, so there is\n"
     "resistance even when stopped - just no centring. Damping resists moving the pedals\n"
     "quickly: a little at rest (the rudder's mass), more with airspeed (air on the rudder).\n"
     "Damping uses CLS2Sim's friction-by-velocity: factor x pedal speed (mm/s or deg/s) = force.",
     [("limp_load_pct", "How heavy the pedals feel when limp (stopped / slow), % of your original\n"
                        "profile. No centring at rest - they stay where you put them."),
      ("damping_rest", "Damping factor when stopped. 0 = none. Try steps of 0.01."),
      ("damping_per_100kt", "Extra damping factor per 100 kt of airspeed."),
      ("damping_limit", "Most force the damping can add (N, or Nm on rotational pedals).")]),
    ("YawFeel",
     "How the pedals react to sideslip (yaw string).",
     [("float_mode", "stream = the rudder floats with the airflow like a real one: wind from\n"
                     "the left (yaw string blown right) eases the right pedal forward; in a slip\n"
                     "you hold, the pushed pedal gets a bit lighter.\n"
                     "resist = the opposite: the pushed pedal pushes back harder the more you\n"
                     "yaw (not physical, but easy to feel)."),
      ("float_gain_per_rad", "Strength. Spring-centre shift per radian of yaw string\n"
                             "(15 deg of yaw ~ 0.26 rad x gain). Higher = more yaw pressure."),
      ("float_max", "Max centre shift, 0-1 of half pedal travel."),
      ("float_fade_start_kt", "No yaw effect below this IAS (kt). Yaw string is noise when slow."),
      ("float_fade_full_kt", "Full yaw effect above this IAS (kt)."),
      ("float_rate_per_s", "How fast the centre can move. Lower = smoother, higher = snappier."),
      ("float_sign", "CALIBRATION ONLY - verified for this rig. Leave at 1.")]),
    ("StallFlutter",
     "With the stick shim installed (install_stick_shim.bat), the pedals flutter with Condor's\n"
     "OWN stall shake - the same moment and strength as the stick, right for every glider and\n"
     "ballast. Without it, the IAS model below is used: high AoA = low airspeed for the g\n"
     "you're pulling, so flutter starts below an IAS that rises with g.",
     [("stall_source", "auto = Condor's stall shake when the stick shim is running, else the IAS\n"
                       "model. condor = only Condor's shake. ias = always the IAS model."),
      ("condor_stall_full_gain", "Condor's shake level (0-10000) that gives full pedal flutter. A\n"
                                 "normal stall peaks around 2600 and settles at 1000-1800 while\n"
                                 "stalled. Lower = pedals reach full flutter sooner."),
      ("condor_stall_curve", "1 = pedal flutter follows Condor's shake linearly. 2 = lighter at\n"
                             "first, full only at Condor's strongest shake."),
      ("buffet_onset_kt", "Flutter onset IAS at 1 g (kt). 0 = off. Set it to the IAS where\n"
                          "the STICK starts to flutter; the pedals then fade in just after."),
      ("buffet_band_kt", "Flutter builds to full strength this many kt below onset."),
      ("buffet_curve", "Ramp shape. 1 = linear. 2 = barely-there at first, then builds to a\n"
                       "full rumble. 3 = even later and more sudden at the end."),
      ("buffet_g_scaling", "true = onset x sqrt(g), so you also get flutter in tight turns."),
      ("buffet_force_max", "Flutter strength at full."),
      ("buffet_interval_ms", "Flutter period (ms). Lower = faster, buzzier shake. Min 10."),
      ("buffet_buzz_force_max", "Extra fast buzz layered on top as you get deep into the stall\n"
                                "(kicks in late, full at the bottom of the band). 0 = off."),
      ("buffet_buzz_interval_ms", "Buzz period (ms).")]),
    ("GroundRumble",
     "Rumble while rolling on the ground. Needs ExtendedData=1 in Condor's UDP.ini.",
     [("ground_vib_enable", "true / false."),
      ("ground_vib_force_max", "Rumble strength at full speed on a normal surface."),
      ("ground_vib_full_kt", "Ground speed where rumble reaches full strength (kt)."),
      ("ground_vib_min_kt", "No rumble below this ground speed (kt)."),
      ("ground_vib_interval_slow_ms", "Rumble period at low speed (ms)."),
      ("ground_vib_interval_fast_ms", "Rumble period at full speed (ms). Lower = faster buzz."),
      ("ground_roughness_gain", "How much Condor's surface roughness changes the rumble.\n"
                                "Strength x (0.5 + gain x roughness), clamped 0.3x-1.5x."),
      ("ground_wheel_height_m", "Wheel height (m) below which you count as on the ground.")]),
    ("Airbrakes",
     "Airbrake buffet on the pedals, stronger with speed. Condor doesn't send airbrake\n"
     "position, so the bridge reads your airbrake lever directly.\n"
     "Run lever_setup.bat once to fill in the lever (device, axis, closed/open values).",
     [("airbrake_enable", "true / false."),
      ("airbrake_device", "The lever's controller (VID_xxxx&PID_xxxx). Set by lever_setup.bat."),
      ("airbrake_axis", "The lever's axis: X Y Z R U or V. Set by lever_setup.bat."),
      ("airbrake_closed_raw", "Raw axis value with airbrakes closed. Set by lever_setup.bat."),
      ("airbrake_open_raw", "Raw axis value with airbrakes fully out. Set by lever_setup.bat."),
      ("airbrake_deadzone", "Lever travel ignored at the closed end (0-0.5)."),
      ("airbrake_vib_force_max", "Buffet strength at full airbrakes at airbrake_vib_full_kt."),
      ("airbrake_vib_full_kt", "IAS where airbrake buffet reaches full strength (kt)."),
      ("airbrake_vib_min_kt", "No airbrake buffet below this IAS (kt)."),
      ("airbrake_vib_low_frac", "Share of the full buffet you still feel at low speed (0-1)."),
      ("airbrake_vib_curve", "Build-up shape. Below 1 = most of it early (60 kt ~ 80% by\n"
                             "default), still growing to full at airbrake_vib_full_kt. 1 = linear."),
      ("airbrake_vib_interval_ms", "Buffet period (ms). Lower = faster, buzzier.")]),
    ("HighSpeedBuffet",
     "Buzz that creeps in at high speed and builds hard toward VNE. VNE varies by glider\n"
     "(roughly 135-150 kt); set hs_buffet_full_kt near the VNE of the glider you fly.",
     [("hs_buffet_onset_kt", "Buffet starts above this IAS (kt). 0 = off."),
      ("hs_buffet_full_kt", "Full strength at this IAS (kt) - put it near VNE."),
      ("hs_buffet_force_max", "Strength at full."),
      ("hs_buffet_curve", "Build-up shape. >1 = creeps in then builds hard near VNE. 1 = linear."),
      ("hs_buffet_interval_ms", "Buzz period (ms). Lower = faster, finer buzz."),
      ("condor_vne_force_max", "With the stick shim: Condor's own VNE flutter (it shakes the stick\n"
                               "just before the wing goes) on the pedals, at full. 0 = off."),
      ("condor_vne_interval_ms", "VNE flutter period (ms).")]),
    ("StickBuffet",
     "With the stick shim installed, the bridge also shakes the STICK for things Condor\n"
     "doesn't: airbrake buffet (same speed curve as the pedals' airbrake buffet) and the\n"
     "high-speed buffet. % of the stick's maximum force. 0 = off.",
     [("stick_airbrake_buffet", "Stick buffet at full airbrakes and airbrake_vib_full_kt."),
      ("stick_hs_buffet", "Stick buffet at hs_buffet_full_kt."),
      ("stick_buffet_period_ms", "Shake period (ms). Lower = faster, finer.")]),
    ("StickGusts",
     "OFF by default - don't turn on. Load factor and roll rate respond to YOUR stick input,\n"
     "so feeding them back as stick force closes a loop through Condor's ~0.1 s of delay:\n"
     "one sign holds the stick off centre, the other makes it slam back and forth.\n"
     "Kept only for experiments; turbulence nudges need an input the stick can't affect.",
     [("stick_gust_pitch_per_g", "% of the stick's max force per 1 g of change. 0 = off."),
      ("stick_gust_roll_per_dps", "% per deg/s of roll-rate change. 0 = off."),
      ("stick_gust_max", "Most force a gust can add, % of max."),
      ("stick_gust_pitch_deadband_g", "Load-factor changes smaller than this (g) add nothing, so the\n"
                                      "stick sits dead still in smooth air instead of wiggling\n"
                                      "around centre. Bigger bumps come through (minus this)."),
      ("stick_gust_roll_deadband_dps", "Same for roll-rate changes (deg/s)."),
      ("stick_gust_pitch_sign", "1 or -1. Pulling up should push the stick forward against your\n"
                                "hand. Wrong sign = the stick 'sticks' wherever you put it near\n"
                                "centre (it holds itself there). -1 on Rebot's CLS-E."),
      ("stick_gust_roll_sign", "1 or -1. Rolling in should push the stick back against your\n"
                               "hand; if it drags it further into the roll, use -1."),
      ("stick_gust_highpass_s", "Changes slower than this (s) are ignored."),
      ("stick_gust_smooth_s", "Smoothing (s) of Condor's data.")]),
    ("StructuralFailure",
     "When a wing comes off (the stick shim sees Condor cut its VNE flutter, or Condor shows a\n"
     "violent, sustained roll at speed), the flutter drops suddenly - but the tail and the other\n"
     "wing are still in the airflow, so some keeps going, fading as you slow down.",
     [("failure_detect", "true / false."),
      ("failure_residual", "Share of the flutter that keeps going after the break (0-1).\n"
                           "0 = everything stops dead (what Condor's own stick does)."),
      ("failure_rollrate_dps", "Roll rate (deg/s) that counts as uncontrolled. Normal flying stays\n"
                               "well under 60; a lost wing rolls at 120-150."),
      ("failure_hold_s", "How long the roll rate must stay that high (s)."),
      ("failure_min_kt", "Only above this IAS (kt), so spins don't count."),
      ("failure_reset_alt_jump_m", "An altitude jump bigger than this (m) means a new flight.")]),
    ("Turbulence",
     "Optional airborne vibration from Condor's turbulence strength. 0 = off.",
     [("turb_vib_force_per_unit", "Vibration force per unit of turbulence."),
      ("turb_vib_interval_ms", "Turbulence vibration period (ms).")]),
    ("Advanced",
     "Leave these alone unless something feels wrong.",
     [("vib_force_cap", "Safety cap on any vibration force sent to the pedals."),
      ("ias_tau_s", "Airspeed smoothing (s). Higher = smoother, laggier stiffness changes."),
      ("yaw_tau_s", "Yaw string smoothing (s). Higher = less twitchy yaw feel."),
      ("stale_after_s", "No new Condor data for this long -> pedals go neutral (s)."),
      ("resend_all_every_s", "Re-assert everything to CLS2Sim this often (s), one item per tick so\n"
                             "it never arrives as a burst. Only matters if you hit Apply in\n"
                             "CLS2Sim's Profile Manager mid-flight. Short intervals (2 s) gave\n"
                             "a periodic stutter in VR."),
      ("tick_hz", "(restart) Update rate (Hz)."),
      ("watchdog_ms", "(restart) 0 = off. >0: CLS2Sim STOPS the pedals if this app dies.")]),
    ("Network",
     "(restart) Connection settings. Changes here need a restart of this app.",
     [("condor_listen_host", "Interface to listen on for Condor UDP."),
      ("condor_listen_port", "Must match Port= in Condor's UDP.ini."),
      ("forward_to", "Relay Condor packets to other apps, comma-separated host:port. Blank = none."),
      ("cls2sim_host", "Machine running CLS2Sim."),
      ("cls2sim_port", "CLS2Sim remote-control UDP port."),
      ("cls2sim_msg_id", "true only if 'Message Identifier' is ticked in CLS2Sim settings."),
      ("cls2sim_timeout_s", "Reply timeout for CLS2Sim requests (s)."),
      ("axis_mask", "Which Brunner axis to drive. 0x4 = rudder."),
      ("shim_udp_port", "UDP port the stick shim sends Condor's shake on (udp_port in\n"
                        "ffb_shim.ini). 0 = ignore the shim."),
      ("shim_cmd_port", "UDP port the stick shim takes buffet commands on (cmd_port in\n"
                        "ffb_shim.ini). 0 = don't send.")]),
]

RESTART_KEYS = {k for sec, _, keys in SETTINGS_LAYOUT if sec == "Network" for k, _ in keys}
RESTART_KEYS |= {"tick_hz", "watchdog_ms"}


def _fmt_value(key, v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, list):
        return ", ".join(v)
    if key == "axis_mask":
        return hex(v)
    return str(v)


def _parse_value(key, raw):
    d = DEFAULTS[key]
    raw = raw.strip()
    if isinstance(d, bool):
        low = raw.lower()
        if low in ("1", "true", "yes", "on"):
            return True
        if low in ("0", "false", "no", "off"):
            return False
        raise ValueError("expected true or false")
    if isinstance(d, int):
        if raw.lower().startswith("0x"):
            return int(raw, 16)
        return int(round(float(raw)))
    if isinstance(d, float):
        return float(raw)
    if isinstance(d, list):
        return [x.strip() for x in raw.split(",") if x.strip()]
    if key == "float_mode" and raw.lower() not in ("resist", "stream"):
        raise ValueError("expected resist or stream")
    if key == "airbrake_axis":
        if raw.upper() not in ("", "X", "Y", "Z", "R", "U", "V"):
            raise ValueError("expected X, Y, Z, R, U or V")
        return raw.upper()
    return raw


def write_settings(path, values=None):
    values = values or DEFAULTS
    lines = [SETTINGS_HEADER]
    for section, blurb, keys in SETTINGS_LAYOUT:
        lines.append("")
        lines.append(f"[{section}]")
        lines += [f"; {ln}" for ln in blurb.split("\n")]
        for key, comment in keys:
            lines.append("")
            lines += [f"; {ln}" for ln in comment.split("\n")]
            lines.append(f"{key} = {_fmt_value(key, values[key])}")
    with open(path, "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(lines) + "\n")


def read_settings(path):
    """Return (values_from_file, warnings). Missing keys fall back to DEFAULTS."""
    cp = configparser.ConfigParser(inline_comment_prefixes=(";", "#"), interpolation=None)
    with open(path, "r", encoding="utf-8-sig") as f:
        cp.read_file(f)
    vals, warns = dict(DEFAULTS), []
    for section in cp.sections():
        for key, raw in cp.items(section):
            if key not in DEFAULTS:
                warns.append(f"unknown setting '{key}' in [{section}] - ignored")
                continue
            try:
                vals[key] = _parse_value(key, raw)
            except ValueError as e:
                warns.append(f"{key} = {raw!r}: {e} - keeping {_fmt_value(key, DEFAULTS[key])}")
    return vals, warns


class SettingsWatcher:
    """Re-reads settings.ini when it changes and applies live-safe keys in place."""

    def __init__(self, path, cfg, file_vals):
        self.path, self.cfg, self.file_vals = path, cfg, file_vals
        self.mtime = self._mtime()

    def _mtime(self):
        try:
            return os.stat(self.path).st_mtime
        except OSError:
            return None

    def check(self):
        m = self._mtime()
        if m is None or m == self.mtime:
            return []
        try:
            new, warns = read_settings(self.path)
        except (OSError, configparser.Error) as e:
            print(f"\n[settings] couldn't read {os.path.basename(self.path)} ({e}); "
                  f"keeping previous values")
            return []                      # mtime not updated -> retried next check
        self.mtime = m
        for w in warns:
            print(f"\n[settings] {w}")
        changed = []
        for k, v in new.items():
            if v == self.file_vals.get(k):
                continue
            if k in RESTART_KEYS:
                print(f"\n[settings] {k} changed - restart the app to apply")
            else:
                changed.append(f"{k} {_fmt_value(k, self.cfg[k])} -> {_fmt_value(k, v)}")
                self.cfg[k] = v
        self.file_vals = new
        return changed


def load_settings_file(path):
    """Load settings.ini, creating it with defaults if missing. Returns (cfg, file_vals)."""
    if not os.path.exists(path):
        write_settings(path)
        print(f"Created {path} with defaults - edit it any time, changes apply live.")
    try:
        vals, warns = read_settings(path)
    except (OSError, configparser.Error) as e:
        print(f"[!] Couldn't read {path} ({e}). Using built-in defaults.")
        return dict(DEFAULTS), dict(DEFAULTS)
    for w in warns:
        print(f"[settings] {w}")
    return dict(vals), dict(vals)


# CLS2Sim remote protocol v2 constants
CMD_READ_DATA = 0xD0
CMD_OVERRIDE = 0xD1
CMD_SIM = 0xD3
CMD_STATE = 0xCD
CMD_SETTING = 0xCE
CMD_RC_CONFIG = 0xD8

OVR_SCALE = 0x01
OVR_TRIM = 0x02
OVR_VIB = 0x05

SET_SCALE = 0x20     # UInt16 20..999
SET_MOVE_BACK = 0x40  # UInt8: 1 = axis returns to centre, 0 = profile acts as friction only
SET_TRIM = 0x90      # float -1..1
SET_VIB_ON = 0xC0    # UInt8
SET_VIB_FAST_MS = 0xC1
SET_VIB_FAST_F = 0xC2
SET_VIB_SLOW_MS = 0xC3
SET_VIB_SLOW_F = 0xC4
SET_FRICTION_BY_VELOCITY = 0xE2  # Float[2] (factor, limit); lost on device state change

DATA_POS_NORM = 0x11

MS_TO_KT = 1.0 / 0.514444   # Condor sends SI units (m/s)

DEVICE_STATES = {0: "not connected", 1: "operational", 2: "not initialized", 3: "ERROR"}


def full_scale(cfg):
    """CLS2Sim scale factor that reproduces the original (undivided) profile."""
    return int(round(clamp(100 * float(cfg["profile_divider"]), 20, 999)))


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class LowPass:
    def __init__(self, tau):
        self.tau = tau
        self.y = None

    def __call__(self, x, dt):
        if self.y is None or self.tau <= 0:
            self.y = x
        else:
            self.y += (dt / (self.tau + dt)) * (x - self.y)
        return self.y

    def reset(self):
        self.y = None


# ----------------------------------------------------------------------------
# Condor UDP input
# ----------------------------------------------------------------------------
class CondorIn:
    """Non-blocking listener for Condor's ASCII 'key=value' UDP stream."""

    def __init__(self, host, port, forward=()):
        # No SO_REUSEADDR on purpose: on Windows it lets two apps share the port and
        # randomly steal each other's packets. Better to fail loudly here.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((host, port))
        except OSError as e:
            sys.exit(f"Can't listen on UDP {host}:{port} ({e}). Is another app using it? "
                     f"Run this bridge there and --forward to the other app instead.")
        self.sock.setblocking(False)
        self.port = port
        self.fwd = []
        for f in forward:
            h, p = f.rsplit(":", 1)
            if int(p) == port and h in ("127.0.0.1", "localhost", "0.0.0.0"):
                sys.exit(f"--forward {f} would loop back into the bridge itself.")
            self.fwd.append((h, int(p)))
        # Separate socket for relaying: a dead forward target makes Windows raise
        # WSAECONNRESET on the *sending* socket; keep that off the listener.
        self.fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.fwd else None
        self.state = {}
        self.packets = 0
        self.last_rx = 0.0
        self.last_change = 0.0     # last time Condor's sim clock moved
        self._last_time_val = None

    def poll(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            for dst in self.fwd:
                try:
                    self.fwd_sock.sendto(data, dst)
                except OSError:
                    pass
            self._parse(data)

    def _parse(self, data):
        now = time.monotonic()
        txt = data.decode("ascii", "replace")
        for line in txt.replace("\r", "\n").split("\n"):
            if "=" not in line:
                continue
            k, v = line.split("=", 1)
            k = k.strip().lower()
            v = v.strip()
            try:
                self.state[k] = float(v)
            except ValueError:
                self.state[k] = v
        self.packets += 1
        self.last_rx = now
        t = self.state.get("time")
        if t is None or t != self._last_time_val:   # sim clock moving = not paused
            self._last_time_val = t
            self.last_change = now


class ShimIn:
    """Condor's own stick shake, from the stick shim (dinput8.dll next to Condor.exe).
    Every 50 ms it sends 'STATE seq=.. stall1=G stall2=G spring_k=K': G is the gain (0-10000)
    Condor gives its pitch (Stall1) and roll (Stall2) shake effects. Both rise together in a
    stall; Stall2 alone is Condor's VNE flutter. -1 = no such effect (yet)."""

    def __init__(self, port, host="127.0.0.1"):
        self.sock = None
        self.stall1 = self.stall2 = self.spring_k = -1
        self.broken = False
        self.last = -1e9
        self.packets = 0
        if port and port > 0:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.bind((host, port))
                s.setblocking(False)
                self.sock = s
            except OSError as e:
                print(f"[shim] can't listen on UDP {port} ({e}) - using the IAS stall model")
                s.close()

    def poll(self):
        while self.sock:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            if b" STATE " not in data:
                continue
            vals = {}
            for tok in data.decode("ascii", "replace").split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    try:
                        vals[k] = int(v)
                    except ValueError:
                        pass
            self.stall1 = vals.get("stall1", -1)
            self.stall2 = vals.get("stall2", -1)
            self.spring_k = vals.get("spring_k", -1)
            self.broken = vals.get("broken", 0) == 1
            self.last = time.monotonic()
            self.packets += 1

    def shake(self, now=None, max_age=1.0):
        """(stall1, stall2, broken) while the shim is alive and Condor has its shake effects,
        else None. broken = the shim saw Condor cut its VNE flutter dead (wing gone)."""
        now = time.monotonic() if now is None else now
        if now - self.last > max_age or self.stall1 < 0 or self.stall2 < 0:
            return None
        return self.stall1, self.stall2, self.broken


# ----------------------------------------------------------------------------
# CLS2Sim remote interface client
# ----------------------------------------------------------------------------
class Cls2Sim:
    """Request/response over UDP. Every send must be followed by a read."""

    def __init__(self, host, port, msg_id=False, timeout=0.25):
        self.addr = (host, port)
        self.msg_id = msg_id
        self._mid = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))
        self._timeout = timeout
        self.sock.settimeout(timeout)
        self.fail_streak = 0
        self.total_fail = 0
        self.last_bad_status = None

    def _drain(self):
        """Drop late replies to requests that already timed out."""
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(8192)
        except OSError:                            # BlockingIOError is an OSError
            pass
        finally:
            self.sock.settimeout(self._timeout)

    def xfer(self, payload):
        """Send one request, return response with the msg-id stripped (or None)."""
        self._drain()
        if self.msg_id:
            self._mid = (self._mid + 1) & 0xFFFFFFFF
            payload = struct.pack("<I", self._mid) + payload
        try:
            self.sock.sendto(payload, self.addr)
            deadline = time.monotonic() + self._timeout
            while True:
                resp, _ = self.sock.recvfrom(8192)
                if self.msg_id:
                    if len(resp) < 6 or struct.unpack_from("<I", resp, 2)[0] != self._mid:
                        if time.monotonic() > deadline:
                            raise socket.timeout()
                        continue
                    resp = resp[:2] + resp[6:]
                if len(resp) < 3:
                    raise socket.timeout()
                self.fail_streak = 0
                if resp[2] != 0:
                    self.last_bad_status = (payload[:12].hex(), resp[:8].hex())
                return resp
        except (socket.timeout, OSError):          # OSError: WSAECONNRESET when nothing listens
            self.fail_streak += 1
            self.total_fail += 1
            return None

    @staticmethod
    def ok(resp):
        return resp is not None and resp[2] == 0

    # --- commands ---
    def override(self, override_id, axes):
        return self.ok(self.xfer(struct.pack("<III", CMD_OVERRIDE, axes, override_id)))

    def setting(self, axes, sid, fmt, *values):
        return self.ok(self.xfer(struct.pack("<III" + fmt, CMD_SETTING, axes, sid, *values)))

    def watchdog(self, ms):
        return self.ok(self.xfer(struct.pack("<IIH", CMD_RC_CONFIG, 0x00, int(ms))))

    def read_pos(self, axes):
        r = self.xfer(struct.pack("<III", CMD_READ_DATA, axes, DATA_POS_NORM))
        if not self.ok(r):
            return None
        out, off = [], 3
        while off + 6 <= len(r):
            dev, pos = struct.unpack_from("<Hf", r, off)
            out.append((dev, pos))
            off += 6
        return out

    def get_status(self, axes):
        r = self.xfer(struct.pack("<III", CMD_STATE, axes, 0x00))
        if not self.ok(r):
            return None
        out, off = [], 3
        while off + 6 <= len(r):
            out.append(struct.unpack_from("<HHH", r, off))   # dev, state, temp
            off += 6
        return out

    def sim_status(self):
        r = self.xfer(struct.pack("<II", CMD_SIM, 0x00))
        if not self.ok(r) or len(r) < 7:
            return None
        _, _, sim_id, connected, rate = struct.unpack_from("<HBBBH", r, 0)
        return sim_id, connected, rate


# ----------------------------------------------------------------------------
# Force model
# ----------------------------------------------------------------------------
JOY_AXES = "XYZRUV"


def _joy_oem_name(vidpid):
    """Friendly controller name from the registry (what joy.cpl shows)."""
    try:
        import winreg
    except ImportError:
        return ""
    sub = ("System\\CurrentControlSet\\Control\\MediaProperties\\PrivateProperties"
           "\\Joystick\\OEM\\" + vidpid)
    for hive in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
        try:
            with winreg.OpenKey(hive, sub) as k:
                return str(winreg.QueryValueEx(k, "OEMName")[0])
        except OSError:
            pass
    return ""


class WinJoy:
    """Minimal Windows game-controller reader (winmm joyGetPosEx). Stdlib only.
    Reads in the background, alongside Condor, without taking the device over."""

    def __init__(self):
        import ctypes
        from ctypes import wintypes
        self.ct = ctypes
        self.winmm = ctypes.WinDLL("winmm")      # raises on non-Windows

        class JOYINFOEX(ctypes.Structure):
            _fields_ = [(n, wintypes.DWORD) for n in (
                "dwSize", "dwFlags", "dwXpos", "dwYpos", "dwZpos", "dwRpos", "dwUpos", "dwVpos",
                "dwButtons", "dwButtonNumber", "dwPOV", "dwReserved1", "dwReserved2")]

        class JOYCAPSW(ctypes.Structure):
            _fields_ = ([("wMid", wintypes.WORD), ("wPid", wintypes.WORD),
                         ("szPname", wintypes.WCHAR * 32)] +
                        [(n, wintypes.UINT) for n in (
                            "wXmin", "wXmax", "wYmin", "wYmax", "wZmin", "wZmax", "wNumButtons",
                            "wPeriodMin", "wPeriodMax", "wRmin", "wRmax", "wUmin", "wUmax",
                            "wVmin", "wVmax", "wCaps", "wMaxAxes", "wNumAxes", "wMaxButtons")] +
                        [("szRegKey", wintypes.WCHAR * 32), ("szOEMVxD", wintypes.WCHAR * 260)])

        self.JOYINFOEX, self.JOYCAPSW = JOYINFOEX, JOYCAPSW
        self.winmm.joyGetPosEx.argtypes = [wintypes.UINT, ctypes.POINTER(JOYINFOEX)]
        self.winmm.joyGetPosEx.restype = wintypes.UINT
        self.winmm.joyGetDevCapsW.argtypes = [ctypes.c_size_t, ctypes.POINTER(JOYCAPSW),
                                              wintypes.UINT]
        self.winmm.joyGetDevCapsW.restype = wintypes.UINT
        self.winmm.joyGetNumDevs.restype = wintypes.UINT

    def read(self, jid):
        ji = self.JOYINFOEX()
        ji.dwSize = self.ct.sizeof(ji)
        ji.dwFlags = 0xFF                          # JOY_RETURNALL
        if self.winmm.joyGetPosEx(jid, self.ct.byref(ji)) != 0:
            return None
        return {"X": ji.dwXpos, "Y": ji.dwYpos, "Z": ji.dwZpos,
                "R": ji.dwRpos, "U": ji.dwUpos, "V": ji.dwVpos}

    def devices(self):
        out = []
        for jid in range(min(16, self.winmm.joyGetNumDevs())):
            if self.read(jid) is None:
                continue
            caps = self.JOYCAPSW()
            name, vidpid, naxes = f"Controller {jid}", "", 6
            if self.winmm.joyGetDevCapsW(jid, self.ct.byref(caps), self.ct.sizeof(caps)) == 0:
                vidpid = f"VID_{caps.wMid:04X}&PID_{caps.wPid:04X}"
                name = _joy_oem_name(vidpid) or caps.szPname or name
                naxes = caps.wNumAxes
            out.append({"id": jid, "name": name, "vidpid": vidpid, "axes": naxes})
        return out


class AirbrakeLever:
    """Airbrake position 0..1 from the lever configured in settings.ini."""

    def __init__(self, cfg, joy_factory=None):
        self.c = cfg
        self.joy_factory = joy_factory or WinJoy
        self.joy = None
        self.jid = None
        self.dev = None
        self.next_try = 0.0
        self.warned = None

    def _warn(self, msg):
        if self.warned != msg:
            print(f"\n[airbrake] {msg}")
            self.warned = msg

    def position(self):
        c = self.c
        if not c["airbrake_enable"] or not c["airbrake_device"] or not c["airbrake_axis"]:
            return 0.0
        if self.dev != c["airbrake_device"]:      # settings changed live -> re-find the lever
            self.jid, self.dev = None, c["airbrake_device"]
        if self.jid is None:
            now = time.monotonic()
            if now < self.next_try:
                return 0.0
            self.next_try = now + 3.0
            try:
                if self.joy is None:
                    self.joy = self.joy_factory()
                match = [d for d in self.joy.devices()
                         if d["vidpid"].upper() == c["airbrake_device"].upper()]
            except Exception as e:
                self._warn(f"can't read game controllers here ({e}) - airbrake buffet off")
                return 0.0
            if not match:
                self._warn(f"lever {c['airbrake_device']} not found - plugged in?")
                return 0.0
            self.jid = match[0]["id"]
            self.warned = None
        r = self.joy.read(self.jid)
        if r is None:
            self.jid = None
            return 0.0
        span = c["airbrake_open_raw"] - c["airbrake_closed_raw"]
        raw = r.get(c["airbrake_axis"])
        if raw is None or span == 0:
            return 0.0
        x = clamp((raw - c["airbrake_closed_raw"]) / span, 0.0, 1.0)
        dz = clamp(c["airbrake_deadzone"], 0.0, 0.5)
        return clamp((x - dz) / (1.0 - dz), 0.0, 1.0)


def update_settings_keys(path, updates):
    """Change 'key = value' lines in settings.ini in place, keeping comments and layout."""
    import re
    with open(path, "r", encoding="utf-8-sig") as f:
        lines = f.read().splitlines()
    for key, val in updates.items():
        pat = re.compile(r"^\s*" + re.escape(key) + r"\s*=")
        for i, ln in enumerate(lines):
            if pat.match(ln):
                lines[i] = f"{key} = {val}"
                break
        else:
            sec = next((i for i, ln in enumerate(lines) if ln.strip() == "[Airbrakes]"), None)
            if sec is None:
                lines += ["", "[Airbrakes]"]
                sec = len(lines) - 1
            lines.insert(sec + 1, f"{key} = {val}")
    with open(path, "w", encoding="utf-8", newline="\r\n") as f:
        f.write("\n".join(lines) + "\n")


def run_lever_setup(settings_path, joy=None):
    print("AIRBRAKE LEVER SETUP")
    try:
        joy = joy or WinJoy()
        devs = joy.devices()
    except Exception as e:
        print(f"Can't read game controllers ({e}). This needs Windows.")
        return False
    if not devs:
        print("No game controllers found.")
        return False
    print("Controllers found:")
    for d in devs:
        print(f"  #{d['id']}: {d['name']}  ({d['vidpid']}, {d['axes']} axes)")
    input("\n1) Put the AIRBRAKE lever fully CLOSED (in), touch nothing else, press Enter...")
    closed = {d["id"]: joy.read(d["id"]) for d in devs}
    input("2) Move it fully OPEN (out), hold it there, press Enter...")
    opened = {d["id"]: joy.read(d["id"]) for d in devs}
    best = None
    for d in devs:
        a, b = closed.get(d["id"]), opened.get(d["id"])
        if not a or not b:
            continue
        for ax in JOY_AXES:
            diff = abs(b[ax] - a[ax])
            if best is None or diff > best[0]:
                best = (diff, d, ax, a[ax], b[ax])
    if not best or best[0] < 3000:
        print("\nDidn't see a lever move far enough. Run it again and use the full travel.")
        return False
    _, d, ax, c_raw, o_raw = best
    print(f"\nAirbrake lever = {d['name']} ({d['vidpid']}), axis {ax}: "
          f"closed {c_raw} -> open {o_raw}")
    update_settings_keys(settings_path, {
        "airbrake_device": d["vidpid"], "airbrake_axis": ax,
        "airbrake_closed_raw": c_raw, "airbrake_open_raw": o_raw})
    print(f"Saved to {settings_path}.")
    print("\nCheck: move the lever for 8 seconds, the bar should follow it.")
    span = o_raw - c_raw
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8:
        r = joy.read(d["id"])
        if r:
            x = clamp((r[ax] - c_raw) / span, 0.0, 1.0)
            bar = "#" * int(x * 30)
            print(f"\r  airbrakes [{bar:30s}] {x * 100:3.0f}%  ", end="", flush=True)
        time.sleep(0.05)
    print("\nDone. If the bridge is running it picks this up within a second.")
    return True


class Targets:
    __slots__ = ("sf", "trim", "fast_f", "fast_ms", "slow_f", "slow_ms", "move_back", "damp")

    def __init__(self, sf, trim, fast_f, fast_ms, slow_f, slow_ms, move_back=1, damp=0.0):
        self.sf, self.trim = sf, trim
        self.fast_f, self.fast_ms = fast_f, fast_ms
        self.slow_f, self.slow_ms = slow_f, slow_ms
        self.move_back = move_back
        self.damp = damp


class ForceModel:
    def __init__(self, cfg):
        self.c = cfg
        self.ias = LowPass(cfg["ias_tau_s"])
        self.yaw = LowPass(cfg["yaw_tau_s"])
        self.trim = 0.0
        self.limp = True                  # start limp: stationary until airflow says otherwise
        self.info = {}
        self.warned_no_ext = False
        self.broken = False               # structural failure latched (wing came off)
        self.fast_roll_s = 0.0
        self.peak_fast = self.peak_slow = 0.0          # recent vibration, for the residual
        self.peak_fast_ms = self.peak_slow_ms = 50
        self.res = None                   # (fast_f, fast_ms, slow_f, slow_ms, kt) frozen at the break
        self.gust_f = None                # band-pass filter state for stick gusts: [g_lp, g_base, r_lp, r_base]
        self.prev_time = None
        self.prev_alt = None

    def _slew(self, target, dt):
        step = self.c["float_rate_per_s"] * dt
        self.trim += clamp(target - self.trim, -step, step)
        return self.trim

    def _gusts(self, s, dt, quiet):
        """Stick gust forces (fractions of max, + = push forward / push right) from quick changes
        in load factor and roll rate: band-passed between stick_gust_smooth_s and _highpass_s."""
        c = self.c
        g = s.get("gforce")
        rr = s.get("rollrate")
        if not isinstance(g, float) or not isinstance(rr, float):
            return 0.0, 0.0
        r = math.degrees(rr)
        if self.gust_f is None:
            self.gust_f = [g, g, r, r]
        f = self.gust_f
        a_lp = dt / (max(1e-3, c["stick_gust_smooth_s"]) + dt)
        a_hp = dt / (max(0.05, c["stick_gust_highpass_s"]) + dt)
        f[0] += a_lp * (g - f[0])
        f[1] += a_hp * (f[0] - f[1])
        f[2] += a_lp * (r - f[2])
        f[3] += a_hp * (f[2] - f[3])
        if quiet:
            return 0.0, 0.0
        cap = max(0.0, c["stick_gust_max"]) / 100.0

        def soft(x, band):                # zero inside the band, continuous outside it
            band = max(0.0, band)
            return math.copysign(max(0.0, abs(x) - band), x)
        p = c["stick_gust_pitch_sign"] * c["stick_gust_pitch_per_g"] / 100.0 * \
            soft(f[0] - f[1], c["stick_gust_pitch_deadband_g"])
        q = -c["stick_gust_roll_sign"] * c["stick_gust_roll_per_dps"] / 100.0 * \
            soft(f[2] - f[3], c["stick_gust_roll_deadband_dps"])
        return clamp(p, -cap, cap), clamp(q, -cap, cap)

    def _latch_broken(self, kt, why):
        if self.broken:
            return
        self.broken = True
        r = clamp(self.c["failure_residual"], 0.0, 1.0)
        self.res = (self.peak_fast * r, self.peak_fast_ms, self.peak_slow * r, self.peak_slow_ms, max(kt, 1.0))
        print(f"\n[structure] wing failure ({why}) - " +
              (f"flutter drops to {r * 100:.0f}% and fades as you slow down" if r > 0
               else "all flutter off") + ", until the next flight")

    def _check_structure(self, s, kt, on_ground, dt):
        """Latch a structural failure: sustained huge roll rate at speed (a wing is gone).
        Reset when Condor starts a new flight (sim time jumps back / altitude teleports)."""
        c = self.c
        t, alt = s.get("time"), s.get("altitude")
        if isinstance(t, float) and self.prev_time is not None and t < self.prev_time - 1e-4:
            self.broken = False
        if (isinstance(alt, float) and self.prev_alt is not None and
                abs(alt - self.prev_alt) > c["failure_reset_alt_jump_m"]):
            self.broken = False
        self.prev_time = t if isinstance(t, float) else self.prev_time
        self.prev_alt = alt if isinstance(alt, float) else self.prev_alt
        rr = s.get("rollrate")
        if (c["failure_detect"] and not on_ground and isinstance(rr, float) and
                kt > c["failure_min_kt"] and abs(math.degrees(rr)) > c["failure_rollrate_dps"]):
            self.fast_roll_s += dt
            if self.fast_roll_s >= c["failure_hold_s"] and not self.broken:
                self._latch_broken(kt, "sustained roll at speed")
        else:
            self.fast_roll_s = 0.0

    def neutral(self, dt):
        c = self.c
        self.broken = False               # menu / new flight
        self.res = None
        self.peak_fast = self.peak_slow = 0.0
        self.ias.reset()
        self.yaw.reset()
        self.info = {"mode": "NEUTRAL"}
        self.limp = True                  # paused / no data: limp, pedals stay put
        div = max(0.2, float(c["profile_divider"]))
        return Targets(int(round(clamp(c["limp_load_pct"] * div, 20, 999))), self._slew(0.0, dt),
                       0, c["ground_vib_interval_slow_ms"], 0, c["turb_vib_interval_ms"],
                       move_back=0, damp=max(0.0, c["damping_rest"]))

    def update(self, s, dt, airbrake=0.0, shake=None):
        """shake: (stall1, stall2) gains from the stick shim, or None."""
        c = self.c
        kt = self.ias(max(0.0, float(s.get("airspeed", 0.0))) * MS_TO_KT, dt)

        # 1) airflow load. No spring: limp below airflow_starts_kt (with hysteresis),
        #    above it centring stiffness follows a power curve in IAS.
        if self.limp and kt > c["airflow_starts_kt"]:
            self.limp = False
        elif not self.limp and kt < c["airflow_starts_kt"] - c["limp_hysteresis_kt"]:
            self.limp = True
        div = max(0.2, float(c["profile_divider"]))
        vr = kt / c["v_ref_kt"]
        load = c["sf_at_vref"] * vr ** c["sf_exponent"]                    # % of original
        knee = max(1.0, c["sf_knee_kt"])
        if kt > knee:      # past the knee: keep building, more gently, up to sf_max
            load = (c["sf_at_vref"] * (knee / c["v_ref_kt"]) ** c["sf_exponent"]
                    * (kt / knee) ** c["sf_exponent_high"])
        load = min(load, c["sf_max"])
        sf = int(round(clamp(load * div, 20, 999)))                         # % of CLS2Sim's
        if self.limp:                     # no centring; the profile acts as linkage friction
            sf = int(round(clamp(c["limp_load_pct"] * div, 20, 999)))
        eff_load = sf / div               # what you actually feel, % of original profile

        # 2) yaw feel: shift the spring centre with sideslip (yaw string), faded in with IAS.
        #    stream: centre moves downwind with the airflow (free rudder floating): string
        #            blown right (+) -> centre toward right pedal forward.
        #    resist: the opposite shift -> force grows with yaw.
        yaw = self.yaw(float(s.get("yawstringangle", 0.0)), dt)
        span = max(1e-6, c["float_fade_full_kt"] - c["float_fade_start_kt"])
        fade = clamp((kt - c["float_fade_start_kt"]) / span, 0.0, 1.0)
        mode = -1.0 if str(c["float_mode"]).lower() == "resist" else 1.0
        tgt = clamp(mode * c["float_sign"] * c["float_gain_per_rad"] * yaw * fade,
                    -c["float_max"], c["float_max"])
        trim = self._slew(tgt, dt)

        # 3) ground roll rumble
        wh = s.get("wheelheight")
        if wh is None and not self.warned_no_ext:
            self.warned_no_ext = True
            print("\n[!] No 'wheelheight' in Condor data -> set ExtendedData=1 in UDP.ini "
                  "for ground effects.")
        on_ground = isinstance(wh, float) and wh < c["ground_wheel_height_m"]
        if all(isinstance(s.get(k), float) for k in ("vx", "vy", "vz")):
            gs = math.sqrt(s["vx"] ** 2 + s["vy"] ** 2 + s["vz"] ** 2) * MS_TO_KT
        else:
            gs = kt
        fast_f, fast_ms = 0, c["ground_vib_interval_slow_ms"]
        if c["ground_vib_enable"] and on_ground and gs > c["ground_vib_min_kt"]:
            frac = clamp(gs / c["ground_vib_full_kt"], 0.0, 1.0)
            r = s.get("surfaceroughness")
            rough = clamp(0.5 + c["ground_roughness_gain"] * r, 0.3, 1.5) \
                if isinstance(r, float) else 1.0
            fast_f = c["ground_vib_force_max"] * frac * rough
            fast_ms = c["ground_vib_interval_slow_ms"] - \
                (c["ground_vib_interval_slow_ms"] - c["ground_vib_interval_fast_ms"]) * frac

        # 4) airborne slow channel: stall flutter and/or turbulence
        slow_f, slow_ms = 0.0, c["turb_vib_interval_ms"]
        stall_pct = 0.0
        hs = 0.0
        stick_buf = 0.0
        g = s.get("gforce")
        g = clamp(g, 0.3, 4.0) if isinstance(g, float) and g > 0 else 1.0
        onset = c["buffet_onset_kt"] * (math.sqrt(g) if c["buffet_g_scaling"] else 1.0)
        src = str(c["stall_source"]).lower()
        use_condor = shake is not None and src in ("auto", "condor")
        stall_src = "condor" if use_condor else ("none" if src == "condor" else "ias")
        vnef = 0.0
        if not on_ground:
            ts = s.get("turbulencestrength")
            turb = c["turb_vib_force_per_unit"] * ts if isinstance(ts, float) else 0.0
            buf = 0.0
            if use_condor:
                s1, s2 = shake[0], shake[1]
                frac = clamp(s1 / max(1.0, float(c["condor_stall_full_gain"])), 0.0, 1.0)
                if frac > 0:
                    buf = c["buffet_force_max"] * frac ** max(0.1, c["condor_stall_curve"])
                    stall_pct = frac * 100
                    buzz = c["buffet_buzz_force_max"] * frac ** 3
                    if buzz > 0:
                        fast_f, fast_ms = buzz, c["buffet_buzz_interval_ms"]
                if s1 < 50:               # roll shake on its own = Condor's VNE flutter
                    vnef = c["condor_vne_force_max"] * clamp(s2 / 10000.0, 0.0, 1.0)
            elif src != "condor" and c["buffet_onset_kt"] > 0 and kt < onset:
                frac = clamp((onset - kt) / max(1e-6, c["buffet_band_kt"]), 0.0, 1.0)
                fade_out = clamp((kt - 5.0) / 10.0, 0.0, 1.0)   # smooth to nothing 15 -> 5 kt
                buf = c["buffet_force_max"] * frac ** max(0.1, c["buffet_curve"]) * fade_out
                stall_pct = frac * 100
                # deep-stall buzz on the (airborne-idle) fast channel, layered on the flutter
                buzz = c["buffet_buzz_force_max"] * frac ** 3 * fade_out
                if buzz > 0:
                    fast_f, fast_ms = buzz, c["buffet_buzz_interval_ms"]
            # high-speed buffet on the fast channel (stall buzz and this never overlap)
            hs = 0.0
            if c["hs_buffet_onset_kt"] > 0 and kt > c["hs_buffet_onset_kt"]:
                span = max(1e-6, c["hs_buffet_full_kt"] - c["hs_buffet_onset_kt"])
                x = clamp((kt - c["hs_buffet_onset_kt"]) / span, 0.0, 1.0)
                hs = c["hs_buffet_force_max"] * x ** max(0.1, c["hs_buffet_curve"])
                if hs > fast_f:
                    fast_f, fast_ms = hs, c["hs_buffet_interval_ms"]
            # airbrake buffet: a little at low speed, building with speed
            abf = 0.0
            if airbrake > 0 and kt > c["airbrake_vib_min_kt"]:
                span = max(1e-6, c["airbrake_vib_full_kt"] - c["airbrake_vib_min_kt"])
                sp = clamp((kt - c["airbrake_vib_min_kt"]) / span, 0.0, 1.0)
                low = clamp(c["airbrake_vib_low_frac"], 0.0, 1.0)
                fade_in = clamp((kt - c["airbrake_vib_min_kt"]) / 5.0, 0.0, 1.0)
                abf = (c["airbrake_vib_force_max"] * airbrake * fade_in *
                       (low + (1 - low) * sp ** max(0.1, c["airbrake_vib_curve"])))
            slow_f, slow_ms = max((buf, c["buffet_interval_ms"]),
                                  (abf, c["airbrake_vib_interval_ms"]),
                                  (vnef, c["condor_vne_interval_ms"]),
                                  (turb, c["turb_vib_interval_ms"]))
            # the stick feels it too (sent to the stick shim)
            ab_frac = abf / max(1.0, float(c["airbrake_vib_force_max"]))
            hs_frac = hs / max(1.0, float(c["hs_buffet_force_max"]))
            stick_buf = clamp(max(c["stick_airbrake_buffet"] / 100.0 * ab_frac,
                                  c["stick_hs_buffet"] / 100.0 * hs_frac), 0.0, 1.0)

        # remember the recent vibration (decaying peak) so a break can keep a share of it
        decay = math.exp(-dt / 1.5)
        if not self.broken:
            if fast_f >= self.peak_fast * decay:
                self.peak_fast, self.peak_fast_ms = fast_f, fast_ms
            else:
                self.peak_fast *= decay
            if slow_f >= self.peak_slow * decay:
                self.peak_slow, self.peak_slow_ms = slow_f, slow_ms
            else:
                self.peak_slow *= decay
        if shake is not None and len(shake) > 2 and shake[2] and not on_ground:
            self._latch_broken(kt, "Condor cut its VNE flutter dead")
        self._check_structure(s, kt, on_ground, dt)
        if self.broken:                   # sudden drop, but the tail and the other wing still flutter
            rf, rfms, rs, rsms, kb = self.res or (0.0, 50, 0.0, 50, 1.0)
            fade = clamp(kt / kb, 0.0, 1.0)
            fast_f, fast_ms = rf * fade, rfms
            slow_f, slow_ms = rs * fade, rsms
            hs = vnef = stick_buf = 0.0
        cap = c["vib_force_cap"]
        gust_p, gust_r = self._gusts(s, dt, on_ground or self.broken)
        self.info = {"mode": "GROUND" if on_ground else "AIR", "kt": kt, "gs": gs,
                     "yaw_deg": math.degrees(yaw), "fade": fade, "g": g, "onset": onset,
                     "limp": self.limp, "load": eff_load,
                     "stall": stall_pct, "ab": airbrake, "hs": hs if not on_ground else 0.0,
                     "broken": self.broken, "src": stall_src, "vne": vnef, "stick_buffet": stick_buf,
                     "gust_pitch": gust_p, "gust_roll": gust_r}
        damp = max(0.0, c["damping_rest"] + c["damping_per_100kt"] * kt / 100.0)
        self.info["damp"] = damp
        return Targets(sf, trim,
                       int(clamp(fast_f, 0, cap)), int(max(1, fast_ms)),
                       int(clamp(slow_f, 0, cap)), int(max(10, slow_ms)),
                       move_back=0 if self.limp else 1, damp=damp)


# ----------------------------------------------------------------------------
# Output stage: only sends what changed (plus periodic full refresh)
# ----------------------------------------------------------------------------
class Output:
    DEADBAND = {"sf": 2, "trim": 0.004, "fast_f": 10, "fast_ms": 3, "slow_f": 10, "slow_ms": 3,
                "move_back": 1, "damp": 0.003}

    def __init__(self, cls, cfg):
        self.cls = cls
        self.c = cfg
        self.axis = cfg["axis_mask"]
        self.sent = {}
        self.vib_on = None
        self.sends = 0
        self.damp_fails = 0               # CLS2Sim versions without friction-by-velocity reject it

    def take_control(self):
        a = self.axis
        ok = all([self.cls.override(OVR_SCALE, a),
                  self.cls.override(OVR_TRIM, a),
                  self.cls.override(OVR_VIB, a)])
        if self.c["watchdog_ms"]:
            ok &= self.cls.watchdog(self.c["watchdog_ms"])
        self.sent.clear()
        self.vib_on = None
        return ok

    def _changed(self, key, val, force):
        old = self.sent.get(key)
        return force or old is None or abs(val - old) >= self.DEADBAND[key]

    def _set(self, key, sid, fmt, val, force):
        if self._changed(key, val, force):
            if self.cls.setting(self.axis, sid, fmt, val):
                self.sent[key] = val
            self.sends += 1

    REFRESH_ORDER = ("control", "sf", "trim", "move_back", "damp", "vib")

    def apply(self, t, force=False):
        """force: True = re-send everything now; a set of REFRESH_ORDER names = re-send just those."""
        if self.c["watchdog_ms"]:
            self.cls.watchdog(self.c["watchdog_ms"])
        f = (lambda name: True) if force is True else (lambda name: bool(force) and name in force)
        if force is not True and f("control"):
            self.take_control()
        self._set("sf", SET_SCALE, "H", int(t.sf), f("sf"))
        self._set("trim", SET_TRIM, "f", float(t.trim), f("trim"))
        self._set("move_back", SET_MOVE_BACK, "B", int(t.move_back), f("move_back"))
        if self.damp_fails < 3 and self._changed("damp", t.damp, f("damp")):
            if self.cls.setting(self.axis, SET_FRICTION_BY_VELOCITY, "ff", float(t.damp),
                                float(self.c["damping_limit"])):
                self.sent["damp"] = t.damp
                self.damp_fails = 0
            else:
                self.damp_fails += 1
                if self.damp_fails == 3:
                    print("\n[!] CLS2Sim didn't accept damping (friction by velocity) - damping off. "
                          "A newer CLS2Sim may support it.")
            self.sends += 1

        want_on = 1 if (t.fast_f > 0 or t.slow_f > 0) else 0
        force = f("vib")
        if want_on and (self.vib_on != 1 or force):
            if self.cls.setting(self.axis, SET_VIB_ON, "B", 1):
                self.vib_on = 1
        if want_on:
            self._set("fast_ms", SET_VIB_FAST_MS, "I", int(t.fast_ms), force)
            self._set("fast_f", SET_VIB_FAST_F, "I", int(t.fast_f), force)
            self._set("slow_ms", SET_VIB_SLOW_MS, "I", int(max(10, t.slow_ms)), force)
            self._set("slow_f", SET_VIB_SLOW_F, "I", int(t.slow_f), force)
        if not want_on and (self.vib_on != 0 or force):
            self._set("fast_f", SET_VIB_FAST_F, "I", 0, True)
            self._set("slow_f", SET_VIB_SLOW_F, "I", 0, True)
            if self.cls.setting(self.axis, SET_VIB_ON, "B", 0):
                self.vib_on = 0

    def release(self):
        """Neutralise and hand everything back to CLS2Sim."""
        a = self.axis
        self.cls.setting(a, SET_VIB_FAST_F, "I", 0)
        self.cls.setting(a, SET_VIB_SLOW_F, "I", 0)
        self.cls.setting(a, SET_VIB_ON, "B", 0)
        self.cls.setting(a, SET_TRIM, "f", 0.0)
        self.cls.setting(a, SET_SCALE, "H", full_scale(self.c))
        self.cls.setting(a, SET_MOVE_BACK, "B", 1)   # back to the profile's normal centring
        if self.damp_fails < 3:
            self.cls.setting(a, SET_FRICTION_BY_VELOCITY, "ff", 0.0, 0.0)
        for oid in (OVR_SCALE, OVR_TRIM, OVR_VIB):
            self.cls.override(oid, 0)            # axis 0 = override off
        if self.c["watchdog_ms"]:
            self.cls.watchdog(0)


# ----------------------------------------------------------------------------
# Modes
# ----------------------------------------------------------------------------
def report_hardware(cls, cfg):
    st = cls.get_status(cfg["axis_mask"])
    if st is None:
        return False
    for dev, state, temp in st:
        if state:
            print(f"  device {dev}: {DEVICE_STATES.get(state, state)}, {temp} C")
    if not any(s for _, s, _ in st):
        print("  [!] No rudder drive reported as connected.")
    sim = cls.sim_status()
    if sim:
        sim_id, conn, rate = sim
        print(f"  CLS2Sim sim id {sim_id} (3 = USB Joystick Emulation), connected={conn}")
        if conn and sim_id != 3:
            print("  [!] CLS2Sim is connected to a simulator. It will fight this bridge. "
                  "Select USB Joystick Emulation.")
    return True


def wait_for_cls2sim(cls, cfg):
    host, port = cfg["cls2sim_host"], cfg["cls2sim_port"]
    shown = False
    while True:
        if report_hardware(cls, cfg):
            return
        if not shown:
            print(f"Waiting for CLS2Sim remote interface on {host}:{port} "
                  f"(Remote control enabled? Low level control OFF? Message Identifier "
                  f"{'ON' if cfg['cls2sim_msg_id'] else 'OFF'}?)")
            shown = True
        time.sleep(2.0)


def run_selftest(cfg):
    cls = Cls2Sim(cfg["cls2sim_host"], cfg["cls2sim_port"], cfg["cls2sim_msg_id"],
                  cfg["cls2sim_timeout_s"])
    print("Self-test: CLS2Sim link, pedal position, trim direction, vibration.")
    wait_for_cls2sim(cls, cfg)
    out = Output(cls, cfg)
    a = cfg["axis_mask"]
    if not out.take_control():
        print(f"  [!] Override command rejected. Last bad reply: {cls.last_bad_status}")
    try:
        print("  Pedal position for 3 s - move the pedals:")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3:
            p = cls.read_pos(a)
            if p:
                print("\r    " + "  ".join(f"dev{d}: {v:+.3f}" for d, v in p) + "      ",
                      end="", flush=True)
            time.sleep(0.1)
        print()
        cls.setting(a, SET_SCALE, "H", full_scale(cfg))
        for val in (+0.25, 0.0, -0.25, 0.0):
            cls.setting(a, SET_TRIM, "f", val)
            print(f"  trim {val:+.2f}  - feet off, note which pedal comes forward")
            time.sleep(2.0)
        print("  LIMP for 6 s (stationary-glider feel) - push the pedals around,")
        print("  they should stay wherever you leave them:")
        cls.setting(a, SET_SCALE, "H", 20)
        cls.setting(a, SET_MOVE_BACK, "B", 0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6:
            p = cls.read_pos(a)
            if p:
                print(f"\r    pos {p[0][1]:+.3f}      ", end="", flush=True)
            time.sleep(0.1)
        print()
        cls.setting(a, SET_SCALE, "H", full_scale(cfg))
        cls.setting(a, SET_MOVE_BACK, "B", 1)
        print("  Centring back on.")
        print("  Gentle vibration (force 60, then 120)")
        cls.setting(a, SET_VIB_ON, "B", 1)
        cls.setting(a, SET_VIB_FAST_MS, "I", 40)
        for f in (60, 120):
            cls.setting(a, SET_VIB_FAST_F, "I", f)
            time.sleep(1.2)
        print("Done. Positive trim = the pedal you saw come forward on +0.25.")
    finally:
        out.release()
    if cls.total_fail:
        print(f"  ({cls.total_fail} requests timed out)")


def run_dump(cfg):
    condor = CondorIn(cfg["condor_listen_host"], cfg["condor_listen_port"], cfg["forward_to"])
    print(f"Listening for Condor on UDP {cfg['condor_listen_port']} ... Ctrl+C to stop")
    last = 0.0
    while True:
        condor.poll()
        now = time.monotonic()
        if now - last >= 1.0:
            last = now
            if not condor.packets:
                print("  (no packets yet - UDP.ini Enabled=1? Host/Port match?)")
                continue
            s = condor.state
            print(f"--- {condor.packets} packets ---")
            for k in sorted(s):
                v = s[k]
                extra = ""
                if k == "airspeed" and isinstance(v, float):
                    extra = f"   ({v * MS_TO_KT:.1f} kt)"
                elif k in ("yawstringangle", "slipball", "bank", "pitch") and isinstance(v, float):
                    extra = f"   ({math.degrees(v):+.2f} deg)"
                print(f"  {k:20s} {v}{extra}")
        time.sleep(0.02)


def beep(kind):
    """Audible cue that works while Condor is full-screen (Windows), else terminal bell."""
    try:
        import winsound
        if kind == "open":
            winsound.Beep(1400, 500)
        elif kind == "close":
            winsound.Beep(550, 220)
            time.sleep(0.08)
            winsound.Beep(550, 220)
        else:                                  # start / done
            for _ in range(3):
                winsound.Beep(1000, 110)
                time.sleep(0.06)
    except Exception:
        print("\a", end="", flush=True)


def analyze_airbrake(rows, cycles):
    """rows: [(t, window_label_or_None, state)]. Returns (ranked candidates, short windows)."""
    win = {}
    for _, lab, st in rows:
        if lab:
            win.setdefault(lab, []).append(st)
    need = [f"C{k}" for k in range(cycles + 1)] + [f"O{k}" for k in range(1, cycles + 1)]
    short = [lab for lab in need if len(win.get(lab, [])) < 5]
    if short:
        return [], short
    keys = set()
    for lab in need:
        for st in win[lab]:
            keys |= {k for k, v in st.items() if isinstance(v, float)}
    keys.discard("time")
    res = []
    for k in keys:
        stats = {}
        for lab in need:
            vals = [st[k] for st in win[lab] if isinstance(st.get(k), float)]
            if len(vals) < 5:
                break
            m = sum(vals) / len(vals)
            stats[lab] = (m, (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5)
        else:
            # open window vs the average of the closed windows either side (cancels drift)
            diffs = [stats[f"O{c}"][0] - (stats[f"C{c-1}"][0] + stats[f"C{c}"][0]) / 2
                     for c in range(1, cycles + 1)]
            md = sum(diffs) / len(diffs)
            if abs(md) < 1e-9:
                continue
            noise = sum(sd for _, sd in stats.values()) / len(stats)
            res.append({
                "key": k,
                "score": abs(md) / (noise + 1e-9 + 1e-6 * abs(md)),
                "cons": sum(1 for d in diffs if d * md > 0 and abs(d) > 0.5 * abs(md)) / len(diffs),
                "closed": sum(stats[f"C{c}"][0] for c in range(cycles + 1)) / (cycles + 1),
                "open": sum(stats[f"O{c}"][0] for c in range(1, cycles + 1)) / cycles,
                "noise": noise,
            })
    res.sort(key=lambda r: (r["cons"] == 1.0, r["score"]), reverse=True)
    return res, []


def run_airbrake_test(cfg, out_path, lead=10.0, open_s=8.0, closed_s=8.0, cycles=3, settle=2.5):
    condor = CondorIn(cfg["condor_listen_host"], cfg["condor_listen_port"], cfg["forward_to"])
    print("AIRBRAKE TEST - finds which Condor UDP field (if any) is the airbrake.")
    print("  Fly straight and wings level with the airbrakes CLOSED, then follow the beeps:")
    print("    3 quick beeps  = test started, keep airbrakes CLOSED")
    print("    1 HIGH beep    = airbrakes fully OUT, hold them there")
    print("    2 LOW beeps    = airbrakes IN, hold them closed")
    print("    3 quick beeps  = done, come back here for the result")
    sched, t = [(0.0, lead, "C0")], lead
    for k in range(1, cycles + 1):
        sched.append((t, t + open_s, f"O{k}"))
        t += open_s
        sched.append((t, t + closed_s, f"C{k}"))
        t += closed_s
    total = t

    print("\nWaiting for Condor data - start a flight (close run_ffb.bat if it's open)...")
    while not (condor.packets and time.monotonic() - condor.last_change < 0.5):
        condor.poll()
        time.sleep(0.02)
    print(f"Got data. Test runs {total:.0f} s.")
    beep("start")
    t0 = time.monotonic()
    rows, cur, seg_start, last_pk = [], None, 0.0, condor.packets
    while True:
        now = time.monotonic() - t0
        if now >= total:
            break
        seg = next((s for s in sched if s[0] <= now < s[1]), sched[-1])
        if seg[2] != cur:
            if cur is not None:
                if seg[2].startswith("O"):
                    print(f"  [{now:4.0f}s] airbrakes OUT")
                    beep("open")
                else:
                    print(f"  [{now:4.0f}s] airbrakes IN")
                    beep("close")
            cur, seg_start = seg[2], seg[0]
        condor.poll()
        tn = time.monotonic()
        if condor.packets != last_pk and tn - condor.last_change < 0.1:   # skip paused frames
            last_pk = condor.packets
            tr = tn - t0
            lab = cur if tr - seg_start >= settle else None   # ignore lever travel / reaction
            rows.append((tr, lab, dict(condor.state)))
        time.sleep(0.005)
    beep("done")

    keys = sorted({k for _, _, st in rows for k in st})
    try:
        import csv
        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "window"] + keys)
            for tr, lab, st in rows:
                w.writerow([f"{tr:.3f}", lab or "-"] + [st.get(k, "") for k in keys])
        saved = True
    except OSError as e:
        print(f"[!] Couldn't save {out_path}: {e}")
        saved = False

    res, short = analyze_airbrake(rows, cycles)
    print("\n================ RESULT ================")
    if short:
        print("Not enough data in some steps (" + ", ".join(short) + "). Was Condor paused?")
    else:
        print(f"{'field':22s} {'closed':>10s} {'open':>10s} {'noise':>9s} {'score':>8s}  in sync")
        for r in res[:6]:
            sc = ">1000" if r["score"] > 1000 else f"{r['score']:.1f}"
            print(f"{r['key']:22s} {r['closed']:10.3f} {r['open']:10.3f} {r['noise']:9.3f} "
                  f"{sc:>8s}  {'yes' if r['cons'] == 1.0 else 'no'}")
        best = res[0] if res else None
        if best and best["cons"] == 1.0 and best["score"] > 25:
            print(f"\n>>> Airbrake field FOUND: '{best['key']}'  "
                  f"(closed ~ {best['closed']:.3f}, open ~ {best['open']:.3f})")
        else:
            print("\n>>> No clean airbrake field. Condor doesn't seem to send it - we'll read")
            print("    your airbrake lever directly instead.")
    if saved:
        print(f"\nFull recording saved to: {out_path}")
        print("Drag that file into the chat if you want me to double-check it.")


def run_record(cfg, out_path):
    """Record every Condor UDP field (+ airbrake lever + controller axes) to CSV.
    Rows are written as they arrive and flushed every second, so closing the window
    loses at most the last second."""
    import csv
    condor = CondorIn(cfg["condor_listen_host"], cfg["condor_listen_port"], cfg["forward_to"])
    print("RECORDING all Condor UDP data. Close run.bat first (same port).")
    lever = AirbrakeLever(cfg)
    shim = ShimIn(cfg["shim_udp_port"])
    joy, joy_ids = None, []
    try:
        joy = WinJoy()
        devs = joy.devices()
        joy_ids = [d["id"] for d in devs]
        for d in devs:
            print(f"  also recording controller #{d['id']}: {d['name']} ({d['vidpid']})")
    except Exception as e:
        print(f"  (not recording controller axes: {e})")
    print("Waiting for Condor data - start a flight...")
    f = w = cols = None
    n, last_pk, t0, last_flush, t_last = 0, 0, None, 0.0, 0.0
    try:
        while True:
            condor.poll()
            if condor.packets == last_pk:
                time.sleep(0.002)
                continue
            last_pk = condor.packets
            now = time.monotonic()
            st = dict(condor.state)
            st["lever_airbrake"] = round(lever.position(), 4)
            shim.poll()
            live_shim = shim.shake(now) is not None
            st["shim_stall1"] = shim.stall1 if live_shim else ""
            st["shim_stall2"] = shim.stall2 if live_shim else ""
            st["shim_spring_k"] = shim.spring_k if live_shim else ""
            for jid in joy_ids:
                r = joy.read(jid)
                if r:
                    for ax, v in r.items():
                        st[f"joy{jid}_{ax}"] = v
            if f is None:
                t0 = now
                cols = sorted(st)
                f = open(out_path, "w", newline="", encoding="utf-8")
                w = csv.writer(f)
                w.writerow(["t_s", "wall_ms"] + cols)
                print(f"Recording to {out_path} ... press Ctrl+C here when done.")
            t_last = now - t0
            w.writerow([f"{t_last:.3f}", int(time.time() * 1000)] + [st.get(k, "") for k in cols])
            n += 1
            if now - last_flush >= 1.0:
                last_flush = now
                f.flush()
                ias = st.get("airspeed")
                ias = f"{ias * MS_TO_KT:5.1f} kt" if isinstance(ias, float) else "?"
                print(f"\r  {t_last:6.1f} s  {n:6d} samples (~{n / max(t_last, 1e-6):3.0f} Hz)  IAS {ias}   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if f:
            f.close()
    print()
    if not n:
        print("Nothing recorded.")
        return
    print(f"Saved {n} samples ({t_last:.0f} s, ~{n / max(t_last, 1e-6):.0f} Hz) to: {out_path}")
    print("I can read it straight from this folder - just tell me it's done.")


def run_bridge(cfg, dry_run=False, watcher=None, max_seconds=0.0):
    condor = CondorIn(cfg["condor_listen_host"], cfg["condor_listen_port"], cfg["forward_to"])
    shim = ShimIn(cfg["shim_udp_port"])
    shim_live = False
    shim_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    model = ForceModel(cfg)
    lever = AirbrakeLever(cfg)
    cls = out = None
    if not dry_run:
        cls = Cls2Sim(cfg["cls2sim_host"], cfg["cls2sim_port"], cfg["cls2sim_msg_id"],
                      cfg["cls2sim_timeout_s"])
        wait_for_cls2sim(cls, cfg)
        out = Output(cls, cfg)
        if not out.take_control():
            print(f"[!] Override rejected by CLS2Sim. Last bad reply: {cls.last_bad_status}")
    print(f"Listening for Condor on UDP {cfg['condor_listen_port']}"
          f"{' -> forwarding to ' + ', '.join(cfg['forward_to']) if cfg['forward_to'] else ''}"
          f"{'  [DRY RUN]' if dry_run else ''}.  Ctrl+C to stop.")
    if watcher:
        print(f"Tuning: {watcher.path}  (save it any time - changes apply live)")

    period = 1.0 / cfg["tick_hz"]
    next_tick = time.monotonic()
    last_full = last_status = last_settings = time.monotonic()
    refresh = []
    connected = True
    pos_txt = ""
    t_start = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if max_seconds and now - t_start >= max_seconds:
                print("\nRun time limit reached.")
                break
            if watcher and now - last_settings >= 1.0:
                last_settings = now
                changed = watcher.check()
                if changed:
                    model.ias.tau = cfg["ias_tau_s"]
                    model.yaw.tau = cfg["yaw_tau_s"]
                    print("\n[settings] applied: " + "; ".join(changed))
            condor.poll()
            shim.poll()
            shake = shim.shake(now)
            if (shake is not None) != shim_live:
                shim_live = shake is not None
                print("\n[shim] " + ("stick shim live - pedals flutter with Condor's own stall shake"
                                     if shim_live else "stick shim quiet - back to the IAS stall model"))
            live = condor.packets and (now - condor.last_change) < cfg["stale_after_s"]
            ab = lever.position()
            t = model.update(condor.state, period, ab, shake) if live else model.neutral(period)
            if shim_live and cfg["shim_cmd_port"] > 0:
                gain = int(round(10000 * model.info.get("stick_buffet", 0.0)))
                gp = int(round(10000 * model.info.get("gust_pitch", 0.0)))
                gr = int(round(10000 * model.info.get("gust_roll", 0.0)))
                try:
                    shim_tx.sendto(f"BUFFET gain={gain} period_ms={int(cfg['stick_buffet_period_ms'])} "
                                   f"pitch={gp} roll={gr}".encode("ascii"),
                                   ("127.0.0.1", int(cfg["shim_cmd_port"])))
                except OSError:
                    pass

            if out:
                if connected:
                    if (now - last_full) >= cfg["resend_all_every_s"]:
                        last_full = now
                        refresh = list(Output.REFRESH_ORDER)    # one item per tick, never a burst
                    out.apply(t, force={refresh.pop(0)} if refresh else False)
                    if cls.fail_streak > 15:
                        connected = False
                        print("\n[!] Lost CLS2Sim. Retrying...")
                else:
                    if cls.get_status(cfg["axis_mask"]) is not None:
                        connected = True
                        out.take_control()
                        out.apply(t, force=True)
                        print("\nCLS2Sim back.")
                    else:
                        time.sleep(1.0)

            if now - last_status >= 0.5:
                last_status = now
                if out and connected:
                    p = cls.read_pos(cfg["axis_mask"])
                    pos_txt = f" pos {p[0][1]:+.2f} |" if p else ""
                i = model.info
                if i.get("mode") == "NEUTRAL":
                    line = "NEUTRAL (no live Condor data)"
                else:
                    ld = i.get("load", t.sf)
                    load = " LIMP" if i.get("limp") else (f"{ld:4.1f}%" if ld < 10 else f"{ld:4.0f}%")
                    line = (f"{i['mode']:6s} IAS {i['kt']:5.1f}kt {i['g']:3.1f}g | load {load} | "
                            f"yaw {i['yaw_deg']:+5.1f}deg -> trim {t.trim:+.3f} | "
                            f"rumble {t.fast_f:3d} | flutter {t.slow_f:3d} "
                            f"stall {i.get('stall', 0):3.0f}% "
                            f"({'condor' if i.get('src') == 'condor' else 'onset %3.0fkt' % i['onset']}) | "
                            f"brakes {i.get('ab', 0) * 100:3.0f}% | hi-spd {i.get('hs', 0):3.0f}"
                            f"{' | VNE flutter %3.0f' % i['vne'] if i.get('vne') else ''}"
                            f" | damp {i.get('damp', 0):.3f}"
                            f"{' | WING GONE' if i.get('broken') else ''}")
                errs = f" err {cls.total_fail}" if cls and cls.total_fail else ""
                print(f"\r{line} |{pos_txt}{errs}   ", end="", flush=True)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if out and connected:
            out.release()
            print("Overrides released, pedals handed back to CLS2Sim.")


def main():
    default_settings = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.ini")
    ap = argparse.ArgumentParser(description="Condor UDP -> Brunner CLS2Sim rudder force feedback")
    ap.add_argument("--settings", default=default_settings,
                    help="settings file (default: settings.ini next to this script)")
    ap.add_argument("--write-settings", metavar="PATH",
                    help="write a fresh settings file with defaults and exit")
    ap.add_argument("--dry-run", action="store_true", help="compute forces, don't talk to CLS2Sim")
    ap.add_argument("--dump", action="store_true", help="print live Condor UDP fields")
    ap.add_argument("--airbrake-test", action="store_true",
                    help="guided (beeping) test that finds Condor's airbrake field, if any")
    ap.add_argument("--lever-setup", action="store_true",
                    help="find your airbrake lever and save it to settings.ini")
    ap.add_argument("--record", action="store_true",
                    help="record all Condor UDP data to condor_record.csv until Ctrl+C")
    ap.add_argument("--selftest", action="store_true", help="exercise the pedals without Condor")
    ap.add_argument("--forward", action="append", default=[], metavar="HOST:PORT",
                    help="relay raw Condor packets to another app (repeatable)")
    ap.add_argument("--condor-port", type=int)
    ap.add_argument("--cls2sim-host")
    ap.add_argument("--cls2sim-port", type=int)
    ap.add_argument("--run-seconds", type=float, default=0.0,
                    help="stop the bridge after this many seconds (for tests)")
    a = ap.parse_args()

    # Some Windows terminals (Git Bash, IDE consoles) block-buffer stdout -> blank window.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    mode = ("selftest" if a.selftest else "dump" if a.dump else
            "airbrake-test" if a.airbrake_test else "record" if a.record else
            "lever-setup" if a.lever_setup else
            "dry-run" if a.dry_run else "bridge")
    print(f"condor_brunner_ffb starting ({mode} mode)", flush=True)

    if a.write_settings:
        write_settings(a.write_settings)
        print(f"Wrote {a.write_settings}")
        return

    cfg, file_vals = load_settings_file(a.settings)
    watcher = SettingsWatcher(a.settings, cfg, file_vals)
    if a.forward:
        cfg["forward_to"] = list(cfg["forward_to"]) + a.forward
    if a.condor_port:
        cfg["condor_listen_port"] = a.condor_port
    if a.cls2sim_host:
        cfg["cls2sim_host"] = a.cls2sim_host
    if a.cls2sim_port:
        cfg["cls2sim_port"] = a.cls2sim_port

    if a.selftest:
        run_selftest(cfg)
    elif a.dump:
        try:
            run_dump(cfg)
        except KeyboardInterrupt:
            pass
    elif a.airbrake_test:
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "airbrake_test.csv")
        try:
            run_airbrake_test(cfg, out)
        except KeyboardInterrupt:
            print("\nTest cancelled.")
    elif a.lever_setup:
        try:
            run_lever_setup(a.settings)
        except (KeyboardInterrupt, EOFError):
            print("\nCancelled.")
    elif a.record:
        run_record(cfg, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "condor_record.csv"))
    else:
        run_bridge(cfg, dry_run=a.dry_run, watcher=watcher, max_seconds=a.run_seconds)


if __name__ == "__main__":
    main()
