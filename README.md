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
| **Stall flutter** | Condor sends no angle of attack, so it uses the physics equivalent: IAS below stall onset × √g. Builds gradually, plus a faster buzz deep in the stall. Works in turns too. |
| **Airbrake buffet** | Condor doesn't send airbrake position, so the bridge reads your airbrake lever directly. A little at low speed, building with speed. |
| **Ground rumble** | Scales with ground speed and Condor's surface roughness. |

Everything is tunable in `settings.ini`, and **changes apply live** while you fly.

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
| `record_flight.bat` | Records everything Condor sends to `condor_record.csv` until Ctrl+C. |

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
- **Vibration** fast/slow (`0xC1`-`0xC4`) - ground rumble, stall buzz / stall flutter, airbrake buffet

When Condor pauses or stops sending, everything goes neutral; on exit all overrides are released.
Protocol reference: <https://cls2sim.brunner-innovation.swiss/protocolv2.htm>

## Development

```
python tests/smoke_test.py
```

Runs the bridge end to end against a fake Condor (`tests/mock_condor.py`, scripted takeoff, slip
and pause) and a fake CLS2Sim (`tests/mock_cls2sim.py`, validates every packet). No hardware needed.

## Roadmap

- **Stick via CLS2Sim** with the same stall and airbrake model, so stick and pedals stay in sync;
  trim on the stick hat.
- **Bridge writes the force profile itself** (`0x10`), removing CLS2Sim's 20% scale-factor floor
  so the load fades smoothly all the way to zero.
- **Stall speed per glider**: detect buffet in Condor's motion data if it's there, otherwise a
  per-glider table plus automatic water-ballast adjustment.
