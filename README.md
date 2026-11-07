# condor-rudder-ffb

Force feedback for **Brunner CLS-E MkII rudder pedals** in **Condor 3**.

Condor only does force feedback on the stick. This bridge reads Condor's UDP flight data and
drives the pedals through **CLS2Sim's remote control interface**, so the rudder finally feels
like it's connected to a control surface in moving air.

```
Condor 3 --UDP (flight data)--> condor_brunner_ffb.py --UDP (protocol v2)--> CLS2Sim --> CLS-E rudder
```

Python 3.8+, standard library only. Windows.

## What you feel

| Effect | How it's modelled |
|---|---|
| **Airflow load** | No spring. Stopped or slow, the pedals are limp and stay where you put them. From ~3 kt the airflow centres them, building with airspeed; above 70 kt (about ¾ of max) it keeps rising more gently up to ~133 kt. |
| **Yaw feel** | The pedal you push pushes back harder the further you yaw (driven by Condor's yaw string). |
| **Stall flutter** | With the stick shim installed, the pedals flutter with **Condor's own stall shake**: same moment and strength as the stick, right for every glider and ballast. Without it: IAS below stall onset × √g. Plus a faster buzz deep in the stall. |
| **Airbrake buffet** | Condor doesn't send airbrake position, so the bridge reads your airbrake lever directly. Already strong by ~60 kt (about 80%), full at 110 kt. |
| **Structural failure** | When a wing comes off (a violent, sustained roll at speed), every buzz and flutter stops instantly, like Condor's stick does, until the next flight. |
| **High-speed buffet** | A fine buzz that creeps in above 115 kt and builds hard toward VNE (145 kt by default; set it per glider). With the shim, Condor's own VNE flutter comes through too. |
| **Ground rumble** | Scales with ground speed and Condor's surface roughness. |

Everything is tunable in `settings.ini`, and **changes apply live** while you fly.

## Stick: stronger Condor force feedback (shim)

Condor's own stick force feedback is good but soft, and Condor has no strength setting. The
`stick_shim` is a small stand-in `dinput8.dll` that sits next to `Condor.exe`. Condor loads it
instead of Windows' DirectInput; it loads the real one and passes everything straight through,
except the force-feedback commands, which it scales by the percentages in `ffb_shim.ini`.
Edits apply within half a second, mid-flight.

What Condor actually sends (measured with the shim's log):

- **Spring**: one spring on both stick axes, stiffness proportional to airspeed (about 104 per kt
  on DirectInput's 0-10000 scale), so it reaches DirectInput's 100% cap around 97 kt. It softens
  sharply in the stall. Offset 0 (no trim through the spring in that flight).
- **Shake**: two fixed triangle waves, *Stall1* (pitch, 159 ms) and *Stall2* (roll, 205 ms, 71%).
  Condor drives them with the effect **gain**, updated at 20 Hz: both ramp up together in a stall
  (peaking around 26% in a Std Cirrus, starting at an equivalent 1-g IAS of 35 kt), and *Stall2*
  alone ramps to 100% over a second for VNE flutter, cutting to zero when the wing breaks.

So the shim scales:

- `spring`: the stiffness. Past the 100% cap an **assist spring** on the same axes carries the
  rest, so the boost still works at high speed (`spring_assist=0` turns that off).
- `shake`: the shake effects' gain, spilling into the magnitude once the gain is at 100%.

- **No input lag**: stick axes and buttons never go through it, only force commands do.
- **Log**: `ffb_shim.log` in the Condor folder lists every effect Condor creates, changes, starts
  and stops (Condor's names from `Effects.ffe`, real Unix-ms timestamps that line up with the
  `wall_ms` column of `record_flight.bat` recordings). `CLIPPED` means the boost hit the stick's
  maximum.
- **Bridge feed**: every 50 ms the shim sends Condor's current shake gains over UDP to
  `127.0.0.1:55291` (`STATE ... stall1= stall2= spring_k=`), plus every log line. The pedal bridge
  uses it for stall flutter and VNE flutter (`stall_source = auto`), and `record_flight.bat`
  records it (`shim_*` columns).
- **Install**: `install_stick_shim.bat` (Condor in `C:\Condor3`; pass another folder as the first
  argument). **Uninstall**: `uninstall_stick_shim.bat` - deletes the one DLL.
- Unknown whether Condor's online file validation minds an extra DLL in its folder: try it offline
  first.

Build: `stick_shim/build.sh` (zig's bundled clang targeting 32-bit Windows, since Condor.exe is
x86). Test: `stick_shim/test/run_wine_test.sh` runs it under 32-bit wine against a fake DirectInput
and checks the scaling, live reload, log and UDP feed.

## Setup

**1. Condor** - `<Condor install>\UDP.ini`

```ini
[General]
Enabled=1
[Connection]
Host=127.0.0.1
Port=55278
[Misc]
SendIntervalMs=20
ExtendedData=1
ExtendedData1=1
LogToFile=0
```

No virtual COM port needed. Keep Condor's own force feedback on for the stick.

**2. CLS2Sim** (tested with 5.9) - *File > Settings*

- Remote control: **enabled**, UDP port **15090**
- Low level control: **off**, Message Identifier: **off**
- Simulation: **USB Joystick Emulation**, sim auto-connect **off**, hardware auto-connect **on**
  (CLS2Sim never "connects" to Condor; it only needs the pedals connected)
- Rudder profile: a normal force profile with move-back enabled. The bridge scales it.

**3. Condor controls** - rudder mapped to the pedals as usual (no vJoy needed on the tested setup).

**4. Airbrake lever (once)** - double-click `lever_setup.bat`, follow the two prompts.

## Running

| File | What it does |
|---|---|
| `run.bat` | Starts the bridge. Leave the window open while flying; Ctrl+C to stop (hands the pedals back to CLS2Sim). |
| `lever_setup.bat` | Finds your airbrake lever and saves it to `settings.ini`. |
| `airbrake_test.bat` | Beep-guided test that checks whether Condor sends an airbrake field. (It doesn't.) |
| `record_flight.bat` | Records everything Condor sends, plus the airbrake lever and every controller axis (stick, trim lever...), to `condor_record.csv` until Ctrl+C. |
| `install_stick_shim.bat` / `uninstall_stick_shim.bat` | Put the stick FFB shim into Condor's folder, or take it out. |

Command line: `python condor_brunner_ffb.py --help` (also `--selftest`, `--dump`, `--dry-run`).
`run.bat` and the test/record tools share UDP port 55278, so run one at a time.

## Tuning

`settings.ini` is created next to the script on first run (`settings.default.ini` is a copy of
the defaults). Every setting has a comment. Edit, save, and the bridge reports what it applied.
Speeds are in knots; stiffness is % of your CLS2Sim rudder profile (more than 100% is fine).

Verified on the original rig: CLS2Sim trim +0.25 moves the right pedal forward, and a left-pedal
slip reads negative on Condor's yaw string, so `float_sign = 1`.

## How it works

The bridge never uses CLS2Sim's low-level control, so CLS2Sim keeps running its own force loop
from your profile. Over the remote interface it overrides, on the rudder axis only:

- **Force scale factor** (`0x20`) - airflow load
- **Move back** (`0x40`) - off when slow, so the pedals go limp
- **Trim position** (`0x90`) - yaw feel
- **Vibration** fast/slow (`0xC1`-`0xC4`) - fast: ground rumble, deep-stall buzz, high-speed buffet;
  slow: stall flutter, airbrake buffet

When Condor pauses or stops sending, everything goes neutral; on exit all overrides are released.
Protocol reference: <https://cls2sim.brunner-innovation.swiss/protocolv2.htm>

## Development

```
python tests/smoke_test.py
python tests/shim_feed_test.py
```

Runs the bridge end to end against a fake Condor (`tests/mock_condor.py`, scripted takeoff, slip
and pause) and a fake CLS2Sim (`tests/mock_cls2sim.py`, validates every packet). No hardware needed.

## Roadmap

- **Extra stick effects through the shim** (airbrake buffet, high-speed buffet) as additional
  DirectInput effects on the stick Condor already drives.
- **Stick via CLS2Sim** with the same effects (stall flutter, airbrake and high-speed buffet), each
  with its own onset and strength per axis - the stick can feel the stall sooner or harder than the
  pedals.
- **Trim as a trim-tab linkage, not a spring.** The trim lever moves the tab, which shifts the
  elevator's zero-force position aerodynamically: no effect at rest, stiffness about the trimmed
  position grows with airspeed, and the stick's free position drifts with speed (speed stability).
- **Bridge writes the force profile itself** (`0x10`), removing CLS2Sim's 20% scale-factor floor
  so the load fades smoothly all the way to zero.
- **Trim through the shim**: check whether Condor moves the spring offset with the trim lever; if not,
  the shim can do the trim-tab model itself.
