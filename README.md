# 48 V / 35 A BLDC hub-motor controller

A KiCad hardware design plus STM32 firmware for a hall-sensored e-bike or scooter hub motor:

* **48 V nominal** (13S Li-ion, 54.6 V full). All power parts are rated 100 V and the TVS clamps at 60 V standoff.
* **35 A battery current limit** and 60 A phase current limit. Both are set in firmware and can be changed over UART.
* **Field-oriented control (FOC)** using the hall sensors, with angle interpolation, for quiet and smooth torque from standstill.
* **Throttle** input (standard 0.8–4.2 V hall throttle).
* **Momentary power button.** Press to switch on, hold for 2 s to switch off. Standby current is a few µA.
* **FWD/REV switch** input (switch to GND = reverse). There is also a **brake** input (switch to GND = cut power, optional regen).
* **UART telemetry and commands** (115200 8N1, 3.3 V). It streams every reading: voltage, currents, power, speed, temperatures, throttle, direction, faults, limits and energy used.
* **Auto-detect** measures motor R and L and learns the hall-sensor table, so any 120° hall hub motor works without setting angles by hand.

```
hardware/
  kicad/                 KiCad project (bldc48.kicad_pro): 4-sheet schematic + routed 4-layer PCB
  renders/               3D renders, routing/layer images, bldc48.glb 3D model
  bldc48-schematic.pdf   schematic as PDF
  bom.csv                bill of materials
  tools/                 circuit.py (the netlist source), generators, autorouting, connectivity checker,
                         render3d/ (VRML export + headless three.js renderer)
firmware/
  src/                   STM32G431 firmware (register-level, no HAL)
  sim/                   closed-loop PC simulation of the firmware against a hub-motor model
  cmsis/                 ST / ARM CMSIS headers (Apache-2.0)
```

> **Status — read this before building.** The schematic is complete and has been checked pin by pin (see
> [Verification](#verification)). The 188 × 74 mm 4-layer PCB (rev C) is **fully routed and passes KiCad DRC with
> 0 violations and 0 unconnected pads**. The power stage is placed by hand (see [Board](#board)); the signal routing
> was done by an autorouter (Freerouting) plus scripted copper pours, so have it reviewed before ordering.
> The firmware compiles and passes a detailed simulation, but it has **not been run on real hardware**. Bring the
> first board up with the current-limited procedure in [First power-up](#first-power-up).

![3D render of the routed board](hardware/renders/bldc48-iso.png)

---

## Block diagram

```
 BAT+ ──┬── TVS ── 8×220µF + 8×2.2µF ──┬──────────────────────────────┐
        │                               │  3 × half bridge             │
        │   ┌─ LM5164 → +12 V ── LM5109B gate drivers ── 6 × IPT015N10N5 ── 0.5 mΩ ── MOTOR A/B/C
        │   │                                                        │      shunt
        ├─Q1┤─ LM5164 → +5 V ── AP2112 → +3.3 V                       INA240A1 ×3 (G=20)
        │   │      (halls, throttle)        │                            │
  PWR ──┘   └── PWR_HOLD ◄──────────── STM32G431CB ◄── I_A/B/C, VBUS, throttle, NTCs
  button                                    │  ▲
                                UART ◄──────┘  └── halls, FWD/REV, brake, button
```

## Connectors

| Ref | Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 |
|---|---|---|---|---|---|---|---|
| J1 / J2 | M5 bolt terminals (left edge, 36 mm apart) | BAT+ / BAT− | | | | | |
| J3–J5 | M5 bolt terminals (bottom edge) | MOTOR A / B / C | | | | | |
| HALL | JST-PH 6 | +5 V | GND | Hall A | Hall B | Hall C | Motor NTC (optional) |
| THROTTLE | JST-PH 3 | +5 V | Signal | GND | | | |
| FWD/REV | JST-PH 2 | DIR (to GND = reverse) | GND | | | | |
| BRAKE | JST-PH 2 | BRAKE (to GND = braking) | GND | | | | |
| POWER_BTN | JST-PH 2 | PWR_SW (momentary to GND) | GND | | | | |
| UART | JST-PH 4 | +3.3 V out | TX (board → host) | RX (host → board) | GND | | |
| SWD | 1×5 header | +3.3 V | SWDIO | SWCLK | NRST | GND | |

The power button and all signal wires carry 3.3–5 V only, never battery voltage, so a cheap handlebar button is fine.
Battery and phase wires should be 12 AWG (10 AWG for long runs) with M5 ring lugs, bolted to the plated terminals
(screw, washer and nut; the terminal pads are via-stitched to the inner planes). Put an **XT90-S anti-spark connector** and a
**40 A fuse** in the battery lead: the board has no reverse-polarity or inrush protection.

## First power-up

1. Flash the firmware over SWD (`cd firmware && make flash`).
2. Use a **bench supply at 48 V with a 2 A current limit**. Connect the UART (3.3 V USB-serial adapter at 115200 baud)
   and the halls, but leave the motor phases disconnected. Press the power button: the green LED blinks and telemetry
   lines (`$TLM,...`) appear. Type `status`. You should see vbus ≈ 48 V and `faults NOT_DETECTED`.
3. Connect the motor phases (any order) and the hall connector. **Lift the wheel** so it spins freely.
4. Type `detect`. The wheel turns slowly forwards and then backwards (about 15 s total). Expected result:
   `detect OK: R = … mOhm, L = … uH, halls learned`. Type `save`.
   *(No UART cable? Hold the brake and full throttle for 3 s right after power-up, then release the throttle. Detect
   runs and saves automatically.)*
5. With the wheel still lifted, apply a little throttle. If the wheel turns the wrong way, `set dir_invert 1` then `save`.
6. Raise the supply limit (or switch to the battery) and set your limits, e.g. `set i_batt_max 35`, `set i_phase_max 60`,
   then `save`.

## UART interface

Lines end with CR or LF. Replies start with `>`. Telemetry lines start with `$TLM` (default every 100 ms;
change with `stream <ms>` or `stream off`).

```
$TLM,ms,state,faults,limits,vbus,ibus,power,ia,ib,ic,id,iq,iq_ref,erpm,rpm,mod,thr_v,thr_pct,dir,brake,hall,t_fet,t_mot,wh,ah,wh_regen
$TLM,105500,IDLE,0000,00,51.99,0.00,0.0,-0.01,0.01,-0.10,0.06,-0.00,0.00,2670,116.1,0.000,0.849,0.0,FWD,0,4,25.0,nan,10.912,0.2224,0.000
```
(Real line from the simulation: a coasting wheel at 116 rpm, 52 V, throttle at rest. `t_mot` is `nan` when no motor
NTC is enabled.)

| Command | What it does |
|---|---|
| `help` | List the commands |
| `status` | Human-readable state, faults, voltage, current, rpm, temperatures and control-loop CPU load |
| `tlm` / `hdr` | Print one telemetry line / print the column names |
| `stream <ms>` / `stream off` | Set the periodic telemetry rate, or turn it off |
| `get [name]` | Show one parameter, or all of them (including the learned hall angles) |
| `set <name> <value>` | Change a parameter in RAM |
| `save` / `defaults` | Write the config to flash / reload the defaults into RAM |
| `detect` / `detect r` / `detect l` / `detect hall` | Run auto-detect: everything, or only R, L or the hall table |
| `clear` | Clear latched faults |
| `off` / `reboot` | Power the board off / restart the MCU |

Main parameters (`get` lists them all): `i_phase_max` 60 A, `i_batt_max` 35 A, `i_batt_regen` 5 A, `i_brake` 0 A (regen
on the brake input, off by default), `v_uv_start` 42 V / `v_uv_cut` 39 V (13S), `v_ov` 60 V, `t_fet_start` 80 °C /
`t_fet_max` 100 °C, `thr_min_v` 1.0 V / `thr_max_v` 4.0 V, `ramp_up` 100 A/s, `erpm_max` 0 (no speed limit),
`pole_pairs` 23 (only affects the rpm display), `mot_ntc_en` 0, `auto_off_min` 0.

| Fault bit (`faults` field, hex) | Meaning | Red LED blinks |
|---|---|---|
| 0x001 | overcurrent (80 A instantaneous phase-current trip) | 1 |
| 0x002 | overvoltage (> `v_ov`) | 2 |
| 0x004 | battery below `v_uv_cut` (no torque; information only) | short blink every 2 s |
| 0x008 | hall sensors invalid (000/111, e.g. unplugged) | 4 |
| 0x010 | throttle wire fault (< 0.4 V or > 4.7 V) | 5 |
| 0x020 | FET over-temperature or NTC broken | 6 |
| 0x040 | motor over-temperature (if `mot_ntc_en`) | 7 |
| 0x080 | halls not learned yet (run `detect`) | 8 |
| 0x100 | last detect failed | 9 |
| 0x200 | current-sensor offset out of range | 10 |
| 0x400 | throttle held at power-up | 11 |

`limits` bits (why torque is reduced right now): 0x01 battery current, 0x02 phase current, 0x04 undervoltage
fold-back, 0x08 FET temperature, 0x10 motor temperature, 0x20 speed limit, 0x40 out of voltage (top speed),
0x80 stall protection.

Faults switch the gates off at once. They clear by themselves once the cause is gone and the throttle is back at
zero, so the motor can never restart while the throttle is held.

## How it works

**Power stage.** Six Infineon IPT015N10N5 (100 V, 1.5 mΩ, TOLL package) are driven by three LM5109B 100 V half-bridge
drivers through 4.7 Ω gate resistors (≈100 ns edges, ≈1.7 W switching loss per phase at 35 A), with 10 k pull-downs on the gates and the PWM inputs. The MCU's TIM1 generates
20 kHz centre-aligned complementary PWM with 400 ns hardware dead time. With the main output disabled, all six gates
are held low.

**Current sensing.** There is a 0.5 mΩ 4-terminal (Kelvin) shunt in each motor phase (Bourns CSS4J-4026R-L500F, 5 W),
read through its separate sense pads by an INA240A1. The INA240A1 has gain 20, rejects the PWM common-mode swing,
accepts −4 to 80 V common mode and is biased to mid-rail. Full scale is ±165 A per phase, comfortably above the 80 A
hard trip (a compile-time check enforces this). Sensing in the phase lines means the current is valid at every moment, including during coasting.

**Control.** The ADCs are triggered by the PWM timer once per period, and the injected-conversion interrupt runs the
FOC loop at 20 kHz:
- Clarke and Park transforms.
- PI current regulators on d and q, with BEMF/cross-coupling feed-forward and conditional-integration anti-windup.
- PWM delay compensation.
- Min/max SVPWM, capped at 92 % so the bootstrap capacitors always recharge.

The rotor angle comes from the halls. Each hall edge gives the exact boundary angle, and between edges the angle is
extrapolated using a speed averaged over a full electrical revolution, which cancels sensor placement error. At
standstill the sector centre is used, which guarantees at least 86 % torque from the very first moment.

**Limits.**
- Throttle sets the torque (iq), with a ramp.
- Phase current limit.
- Battery current limit, from the power balance `I_bat = 1.5·vq·iq / Vbus`. In simulation it holds 34.8 A against a 35 A setting.
- Undervoltage, FET/motor temperature, speed and stall fold-backs.
- Instant 80 A hard trip and overvoltage trip.
- A direction change is only accepted once the wheel has stopped.

The motor flux linkage is learned while riding and used to re-engage smoothly at speed.

**Power latch.** When the button is pressed it pulls Q1's base (MMBTA92, 300 V PNP) low through 47 k and a diode.
Q1 enables both LM5164 bucks. The MCU then sets PWR_HOLD, which turns on Q2 (MMBTA42) to keep Q1 on. A BAT46W
blocks the button-sense pull-up, so the latch cannot creep on through the unpowered 3.3 V rail. The bucks' EN divider
also gives a 20 V UVLO.

**Supplies.** LM5164 (100 V synchronous buck, 300 kHz COT) for 48→12 V gate drive and 48→5 V sensors; AP2112K
5→3.3 V logic. `Fsw = Vout·2500/Ron(kΩ)`, `Vout = 1.2·(1+Rtop/Rbot)`, type-3 ripple injection sized for 25 mV at FB.

## Verification

* `hardware/tools/check_netlist.py` exports KiCad's own netlist and checks that **every pin of all 172 parts** sits on
  the intended net, with no single-node nets. KiCad 7's CLI has no ERC, so this replaces it. A deliberately miswired
  pin is caught.
* KiCad DRC on the routed PCB: **0 violations, 0 unconnected pads, 0 footprint errors**, including the check that
  every footprint matches the stock KiCad library (report: `hardware/kicad/routing/drc.rpt`).
* `cd firmware && make sim` runs the **real** `motor.c`/`hall.c`/`cli.c` in closed loop against a model with:
  dead-time, PWM delay, ADC noise and offsets, 120° halls at an arbitrary offset, a 100 kg rider and a battery with
  internal resistance. It does this for two different motors. Results for motor 1:

  | Check | Result |
  |---|---|
  | Detect R / L / hall angles | 119.9 mΩ (120) / 259 µH (250) / 0.3° worst error |
  | Start from standstill, full throttle | torque ≥ 86 % of command from the first instant |
  | Peak phase current | 60.2 A (limit 60 A) |
  | Peak battery current (100 ms avg) | 34.7 A (limit 35 A) |
  | 120 A phase-short spike | hard trip, gates off within 200 µs |
  | Current-loop tracking | 0.49 A rms |
  | Top speed, 26″ wheel | ≈ 40 km/h (voltage limited) |
  | Coast, re-engage at speed, reverse, brake, throttle-wire break, hall unplug, low battery, overvoltage | all pass |

## Building the firmware

```
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi   # plus stlink-tools or openocd
cd firmware
make            # build/bldc48.elf/.bin/.hex  (~37 KB flash)
make flash      # st-flash, or: make flash-ocd
make sim        # closed-loop simulation on the PC
```

## Regenerating the hardware

The schematic, PCB and BOM are generated from `hardware/tools/circuit.py`, using symbols and footprints from the stock
KiCad libraries. After you edit the circuit, run `hardware/tools/regen.sh`. It needs KiCad ≥ 7; KiCad 8 and 9 open the
files directly.

`regen.sh` only places the parts. The board was then routed with
`hardware/tools/route_pcb.py --legacy --no-optimize --freerouting freerouting-1.9.0.jar`. This script:
- keeps tracks off the inner layers under the FETs, DC-link caps and shunts (rule area)
- routes the signals with Freerouting (headless, under `xvfb-run`)
- imports the result (KiCad 7 can only import a Specctra session from its GUI, so the script parses the file itself,
  restores any layer-change vias the router left out, adds the recorded hand routes in `HAND_ROUTES`, then tries
  short DRC-checked candidate routes for anything still open)
- adds the GND / +48V / +3V3 planes, the 35 A outer-layer pours and via arrays in the power pads
- fills the zones and writes a DRC report

The routing session is saved in `hardware/kicad/routing/bldc48.ses`, so `route_pcb.py --ses
../kicad/routing/bldc48.ses` rebuilds the routed board in about a minute without running the autorouter.

`hardware/tools/render3d/render.sh` exports the board to VRML with the KiCad 3D models and renders the PNGs in
`hardware/renders/` with three.js in headless Chromium.

**Note:** regenerating overwrites placement and routing. For changes after this point, edit the board in KiCad and use
Tools → Update PCB from Schematic. The footprints are already linked to their schematic symbols for that.

### Board

| Overview | Power stage |
|---|---|
| ![overview](hardware/renders/bldc48-iso.png) | ![power stage](hardware/renders/bldc48-power.png) |
| ![top](hardware/renders/bldc48-top.png) | ![controller](hardware/renders/bldc48-logic.png) |

Routing (tracks only, pours hidden): ![routing](hardware/renders/bldc48-routing.png)

Copper layers: [top](hardware/renders/bldc48-layer-top.png) · [inner 1, GND](hardware/renders/bldc48-layer-in1.png) ·
[inner 2, +48V/+3V3](hardware/renders/bldc48-layer-in2.png) · [bottom](hardware/renders/bldc48-layer-bottom.png)

Terminals close-up: ![terminals](hardware/renders/bldc48-terminals.png)

Stackup as built: F.Cu signals + pours (switch nodes, phase outputs, +48V bus, GND fill) · In1.Cu GND plane ·
In2.Cu +48V under the power stage and +3V3 under the logic · B.Cu signals + the same high-current pours. The inner
planes are unbroken under the FETs, DC-link caps and shunts. 1776 track segments, 453 vias, of which 304 are arrays
in the FET, shunt and DC-link capacitor pads tying the outer pours and the planes together.

## Rev C: compact layout

Rev C has the same circuit as rev B on a smaller board: **188 × 74 mm instead of 240 × 100 mm (42 % less area)**.
- The three half-bridge columns are 38 mm wide instead of 50 mm, and the gate driver, shunt and INA240 of each phase
  pack around its FET pair.
- The 220 µF caps sit in one row along the top edge, right on the +48 V pour.
- The buck inductors are 8 × 8 mm Bourns SRN8040TA instead of 12 × 12 mm SRR1260. The 12 V and 5 V loads are only
  a few hundred mA, so the smaller parts still have plenty of saturation margin.
- The logic, supplies and connectors fill a 52 mm strip on the right.
- The M5 battery and motor terminals keep their positions relative to the bridges: battery on the left edge, each
  motor terminal straight below its shunt.

The price of the smaller board is less copper. Each phase output is a 7.3 mm pour on both outer layers (2 oz), about
35 mm long. By IPC-2221 that carries about 55 A at a 30 °C rise, which covers the 60 A peak / ≈ 42 A rms phase limit,
but with less margin than rev B. Keep the 2 oz outer copper and the heat-spreader plate.

## Ordering from JLCPCB (rough cost)

These are estimates from JLCPCB's published pricing and LCSC part prices in October 2026. Upload the files to get a
real quote, because their prices change often.

| Item (order of 5 assembled boards) | Approx. |
|---|---|
| 4-layer PCB 188 × 74 mm, 1.6 mm, 2 oz outer copper, 5 pcs | $40–70 |
| PCBA setup + stencil (economic) | ≈ $10 |
| "Extended" LCSC part loading fee, ≈ $3 per unique part type (~25 types) | ≈ $75 |
| SMT + through-hole joints (≈ 700 SMT + ≈ 60 THT per board) | ≈ $15 |
| Components (BOM ≈ $32/board) | ≈ $160 |
| Shipping (DHL / FedEx) | $20–35 |
| **Total for 5** | **≈ $290–380, about $58–76 per board** |

- JLCPCB only assembles from 2 boards upward, and the setup and part-loading fees are charged once per order. A
  2-board order is still about $200, so most of the cost of a small order is fees.
- The shunts are Bourns CSS4J-4026R-L500F (LCSC/JLCPCB C2076423, 0.5 mΩ ±1 %, 5 W, 4-terminal, 10.06 × 6.60 mm),
  which JLCPCB can place. The footprint is KiCad's `R_Shunt_Isabellenhuette_BVR4026`: Bourns' recommended land
  pattern has the same 10.6 mm span, 5.6 mm current pads and 0.9 mm sense pads, so the Isabellenhütte BVR 4026 also
  fits as a drop-in alternative. Check the pad drawing against the Bourns datasheet before ordering. Avoid 3 % parts such as
  the Milliohm HOVB4026-5W-0.5mR-3%: they add up to 3 % gain error to every current reading.
- Check that every part you order has a 100 V rating where the BOM needs one: the TOLL FETs, LM5109B, LM5164, the
  220 µF/100 V caps and the 100 V ceramics. Do not let the assembly service substitute lower-voltage parts.
- Set the stackup to 2 oz outer copper. JLCPCB's cheap 4-layer offer is 1 oz.

## Design review (rev B)

A full review of rev A found and fixed these problems:

| Problem in rev A | Fix in rev B |
|---|---|
| BAT+ / BAT− were 5×10 mm SMD solder pads 7.5 mm apart (≈2.5 mm copper gap), 6 mm in from the edge; SMD pads tear off under 12 AWG wire strain | M5 plated bolt terminals for ring lugs, via-stitched to the planes, on the left edge 36 mm apart, TVS between them, large BAT+/BAT− silkscreen |
| Motor pads sat 38 mm below their shunts | M5 bolt terminals on the bottom edge straight below each shunt, labelled MOTOR A/B/C |
| **Overcurrent trip could never fire**: INA240A2 (gain 50) saturated the ADC at ±66 A, below the 80 A trip | INA240A1 (gain 20, ±165 A full scale); a compile-time assert keeps the trip inside the measurable range; new simulation test trips on a 120 A spike within 200 µs |
| INA240 inputs tapped the switch-node/phase copper anywhere, so pour resistance corrupted the current reading | 4-terminal Kelvin shunt (Bourns CSS4J-4026R-L500F, 5 W) with dedicated sense pads to the amplifier |
| FET temperature sensor TH1 was in the board corner, far from the FETs | TH1 next to the middle low-side FET |
| DC-link ceramics ~32 mm from the low-side sources (large switching loop) | Ceramics directly under each low-side FET's source leads; both FETs drain-tab up so the switch node is a short gap between them |
| Low-side gate on the far side of the FET from its driver | Driver beside the FET pair, both gate pins facing it |
| Router ran signals on In2 straight under the FETs, slotting the +48V plane between the drains and the ceramics | Rule area keeps tracks off both inner layers under the FETs, ceramics and shunts |
| 10 Ω gate resistors: ≈170 ns edges, ≈2.9 W switching loss per high-side FET at 35 A | 4.7 Ω: ≈100 ns edges, ≈1.7 W |
| 6 × 220 µF carried ≈2.8 A rms ripple each | 8 × 220 µF, ≈2.1 A each |
| Pours sized from pad centres missed the TOLL drain tabs' real outline (vias in the tabs connected on one layer only) | Pours drawn from the measured pad outlines; inner planes solid-connected to the power terminals |

Checks run on rev B: every symbol pin matches its footprint pad (29 symbol/footprint pairs), every pin of all 174
parts is on its intended net, every footprint matches the stock KiCad library, KiCad DRC 0 violations, firmware
simulation passes on two motors including the new overcurrent test.

## PCB layout rules

* Use 4 layers with 2 oz outer copper. Run 35 A battery and phase paths as wide pours on two or more layers,
  stitched with many vias. Rev C's phase pours are 7.3 mm wide on both outer layers. Do not narrow them further.
* Keep each half-bridge loop (high FET → low FET → 2.2 µF ceramics) as small as possible. Put the 220 µF caps right
  next to the bridges.
* Give the FET drain/source pads thermal-via arrays down to a bottom pour. Bolt the board to an aluminium plate or the
  case through a thermal pad. At 35 A, expect roughly 6–10 W of losses at full load.
* Route each INA240's IN+/IN− from the shunt's own sense pads (pins 2/3 of the 4-terminal shunt), as a tight pair.
* Keep the gate-drive loops short (driver → 4.7 Ω → gate, source → HS/VSS). Use one solid ground plane, and keep the
  MCU and analog parts on the side away from the switching nodes.
* Keep TH1 (the NTC) next to the low-side FETs, which run hottest.
* Keep signal tracks off the inner planes under the FETs, DC-link caps and shunts (enforced by a rule area).

## Things to double-check against datasheets before ordering

* The LM5164 ripple-injection and inductor values were chosen with the datasheet formulas but not run through TI
  WEBENCH.
* The bulk capacitor ripple rating: ≈ 17 A rms total at 35 A means about 2.1 A per cap for 8 caps. Use high-ripple
  parts, or add caps.
* Choose `v_uv_*` for your pack. The defaults are for 13S.
