"""
Netlist-level description of the 48 V / 35 A hub-motor controller.

Every component is described once here.  gen_schematic.py turns this into a
hierarchical KiCad 7 schematic (symbols pulled straight from the stock KiCad
libraries so the pin-outs are the library's, not hand-drawn), and gen_pcb.py
turns the same data into a placed PCB with nets assigned.

Pin keys can be the pin *number* (str) or the pin *name* as it appears in the
KiCad library symbol.  A net name of None marks the pin as no-connect.
Nets whose names match a power symbol (GND, +48V, +12V, +5V, +3V3) are drawn
with power symbols; everything else gets a net label.
"""

POWER_NETS = {'GND', '+48V', '+12V', '+5V', '+3V3'}

SHEETS = [
    ('power', 'Power input, soft-latch, supplies'),
    ('bridge', '3-phase power stage and current sensing'),
    ('mcu', 'STM32G431 controller'),
    ('io', 'Connectors and I/O conditioning'),
]

components = []
_counters = {}


def _ref(prefix):
    _counters[prefix] = _counters.get(prefix, 0) + 1
    return f'{prefix}{_counters[prefix]}'


def part(sheet, prefix, lib_id, value, footprint, pins, **fields):
    ref = _ref(prefix)
    components.append(dict(ref=ref, sheet=sheet, lib_id=lib_id, value=value,
                           footprint=footprint, pins=pins, fields=fields))
    return ref


# ----------------------------------------------------------------------------
# Footprints
FP_R0603 = 'Resistor_SMD:R_0603_1608Metric'
FP_R2512 = 'Resistor_SMD:R_2512_6332Metric'
FP_C0603 = 'Capacitor_SMD:C_0603_1608Metric'
FP_C0805 = 'Capacitor_SMD:C_0805_2012Metric'
FP_C1210 = 'Capacitor_SMD:C_1210_3225Metric'
FP_CBULK = 'Capacitor_THT:CP_Radial_D12.5mm_P5.00mm'
FP_SOD123 = 'Diode_SMD:D_SOD-123'
FP_SMA = 'Diode_SMD:D_SMA'
FP_SMC = 'Diode_SMD:D_SMC'
FP_SOT23 = 'Package_TO_SOT_SMD:SOT-23'
FP_LED = 'LED_SMD:LED_0603_1608Metric'
# Power terminals: M5 plated bolt-down pads (ring lug on 10-12 AWG), via-stitched
# to the inner planes.  SMD solder pads tear off under wire strain at 35 A.
FP_TERMINAL = 'MountingHole:MountingHole_5.3mm_M5_Pad_Via'
FP_SHUNT = 'Resistor_SMD:R_Shunt_Isabellenhuette_BVR4026'
FP_FB = 'Inductor_SMD:L_0603_1608Metric'


def jst_ph(n):
    return f'Connector_JST:JST_PH_B{n}B-PH-K_1x{n:02d}_P2.00mm_Vertical'


def R(sheet, value, a, b, fp=FP_R0603, **kw):
    return part(sheet, 'R', 'Device:R', value, fp, {'1': a, '2': b}, **kw)


def C(sheet, value, a, b, fp=FP_C0603, **kw):
    return part(sheet, 'C', 'Device:C', value, fp, {'1': a, '2': b}, **kw)


def CP(sheet, value, pos, neg, fp=FP_CBULK, **kw):
    return part(sheet, 'C', 'Device:C_Polarized', value, fp, {'1': pos, '2': neg}, **kw)


def D(sheet, value, anode, cathode, fp=FP_SOD123, **kw):
    # Device:D -> pin 1 = K, pin 2 = A
    return part(sheet, 'D', 'Device:D', value, fp, {'1': cathode, '2': anode}, **kw)


def LED(sheet, value, anode, cathode, **kw):
    return part(sheet, 'D', 'Device:LED', value, FP_LED, {'1': cathode, '2': anode}, **kw)


def CONN(sheet, n, nets, fp=None, value=None, **kw):
    lib = f'Connector_Generic:Conn_01x{n:02d}'
    return part(sheet, 'J', lib, value or f'Conn_01x{n:02d}', fp or jst_ph(n),
                {str(i + 1): net for i, net in enumerate(nets)}, **kw)


# ============================================================================
# POWER SHEET
# ============================================================================
S = 'power'

# Battery input: M5 bolt terminals at the board edge, 36 mm apart (ring lugs on
# 10-12 AWG; put an XT90-S anti-spark plug and a 40 A fuse in the battery lead)
CONN(S, 1, ['+48V'], fp=FP_TERMINAL, value='BAT+', MPN='M5 ring lug terminal')
CONN(S, 1, ['GND'], fp=FP_TERMINAL, value='BAT-', MPN='M5 ring lug terminal')
part(S, 'D', 'Device:D_TVS', 'SMCJ60CA', FP_SMC, {'1': '+48V', '2': 'GND'})

# --- Soft power latch ---------------------------------------------------------
# Momentary switch pulls PWR_SW to GND (switch only ever sees <3.3 V) -> current
# flows out of Q1 base through R(47k) + D -> Q1 (high-voltage PNP) turns on -> BUCK_EN rises -> both bucks
# start -> MCU boots and drives PWR_HOLD high (Q2 keeps Q1 on).  To switch off
# the MCU sees a long press on PWR_BTN, waits for release, drops PWR_HOLD.
part(S, 'Q', 'Transistor_BJT:MMBTA92', 'MMBTA92', FP_SOT23,
     {'B': 'LATCH_B', 'E': '+48V', 'C': 'EN_RAW'})
R(S, '100k', '+48V', 'LATCH_B')
R(S, '47k', 'LATCH_B', 'LATCH_PULL')
D(S, '1N4148W', 'LATCH_PULL', 'PWR_SW')
part(S, 'Q', 'Transistor_BJT:MMBTA42', 'MMBTA42', FP_SOT23,
     {'B': 'HOLD_B', 'E': 'GND', 'C': 'LATCH_PULL'})
R(S, '10k', 'PWR_HOLD', 'HOLD_B')
R(S, '100k', 'HOLD_B', 'GND')
# Button sense: the Schottky blocks any DC path from PWR_SW into the (unpowered)
# 3V3 rail, otherwise Q1 base current could leak through it and self-start.
R(S, '10k', '+3V3', 'PWR_BTN')
D(S, 'BAT46W', 'PWR_BTN', 'PWR_SW')
C(S, '10n', 'PWR_BTN', 'GND')
R(S, '100k', 'EN_RAW', 'GND')
# EN/UVLO divider: bucks only start above ~20 V
R(S, '330k', 'EN_RAW', 'BUCK_EN')
R(S, '27k', 'BUCK_EN', 'GND')
C(S, '1n', 'BUCK_EN', 'GND')

# --- 12 V gate-drive buck (LM5164, 100 V / 1 A, COT) --------------------------
# Fsw(kHz) = Vout*2500/Ron(k) -> 12 V, Ron=100k -> 300 kHz
# Vout = 1.2 V * (1 + 90.9k/10k) = 12.1 V
C(S, '2.2u/100V', '+48V', 'GND', fp=FP_C1210)
C(S, '100n/100V', '+48V', 'GND', fp=FP_C0805)
part(S, 'U', 'Regulator_Switching:LM5164DDA', 'LM5164DDA',
     'Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias',
     {'VIN': '+48V', 'EN/UVLO': 'BUCK_EN', 'RON': 'RON12', 'GND': 'GND',
      'EP': 'GND', 'BST': 'BST12', 'SW': 'SW12', 'FB': 'FB12', 'PGOOD': None})
R(S, '100k', 'RON12', 'GND')
C(S, '2.2n', 'BST12', 'SW12')
# 12 V load is ~0.1 A (gate drive): peak = load + ripple/2 = ~0.35 A
part(S, 'L', 'Device:L', '68u', 'Inductor_SMD:L_Bourns_SRN8040TA',
     {'1': 'SW12', '2': '+12V'}, MPN='Bourns SRN8040TA-680M or eq. (68 uH, Isat >= 0.8 A, 8x8 mm)')
R(S, '90.9k', '+12V', 'FB12')
R(S, '10k', 'FB12', 'GND')
# Type-3 ripple injection: RA = (Vin-Vout)*ton/(25mV*CA)
R(S, '365k', 'SW12', 'RIP12')
C(S, '3.3n', 'RIP12', '+12V')
C(S, '100n', 'RIP12', 'FB12')
C(S, '22u/25V', '+12V', 'GND', fp=FP_C1210)
C(S, '22u/25V', '+12V', 'GND', fp=FP_C1210)

# --- 5 V logic / sensor buck -------------------------------------------------
# Ron = 5*2500/300 = 41.7k -> 41.2k ; Vout = 1.2*(1+31.6/10) = 4.99 V
C(S, '2.2u/100V', '+48V', 'GND', fp=FP_C1210)
C(S, '100n/100V', '+48V', 'GND', fp=FP_C0805)
part(S, 'U', 'Regulator_Switching:LM5164DDA', 'LM5164DDA',
     'Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias',
     {'VIN': '+48V', 'EN/UVLO': 'BUCK_EN', 'RON': 'RON5', 'GND': 'GND',
      'EP': 'GND', 'BST': 'BST5', 'SW': 'SW5', 'FB': 'FB5', 'PGOOD': None})
R(S, '41.2k', 'RON5', 'GND')
C(S, '2.2n', 'BST5', 'SW5')
part(S, 'L', 'Device:L', '33u', 'Inductor_SMD:L_Bourns_SRN8040TA',
     {'1': 'SW5', '2': '+5V'}, MPN='Bourns SRN8040TA-330M or eq. (33 uH, Isat >= 1 A, 8x8 mm)')
R(S, '31.6k', '+5V', 'FB5')
R(S, '10k', 'FB5', 'GND')
R(S, '178k', 'SW5', 'RIP5')
C(S, '3.3n', 'RIP5', '+5V')
C(S, '100n', 'RIP5', 'FB5')
C(S, '22u/10V', '+5V', 'GND', fp=FP_C1210)
C(S, '22u/10V', '+5V', 'GND', fp=FP_C1210)

# --- 3.3 V LDO ---------------------------------------------------------------
part(S, 'U', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3', 'Package_TO_SOT_SMD:SOT-23-5',
     {'VIN': '+5V', 'EN': '+5V', 'GND': 'GND', 'VOUT': '+3V3'})
C(S, '1u', '+5V', 'GND')
C(S, '4.7u', '+3V3', 'GND', fp=FP_C0805)
R(S, '2.2k', '+3V3', 'PWR_LED_A')
LED(S, 'GREEN', 'PWR_LED_A', 'GND')

# Power flags (tell ERC these nets are driven)
for net in ['+48V', 'GND', '+12V', '+5V']:
    part(S, '#FLG', 'power:PWR_FLAG', 'PWR_FLAG', '', {'1': net})

# ============================================================================
# BRIDGE SHEET
# ============================================================================
S = 'bridge'

# DC-link capacitance: 8 x 220 uF/100 V low-ESR (~17 A rms ripple at 35 A ->
# ~2.1 A per cap) + 6 x 2.2 uF/100 V X7R right at the low-side FET sources
for i in range(8):
    CP(S, '220u/100V', '+48V', 'GND', MPN='Panasonic EEU-FS2A221 or eq. low-ESR, >=2.1 A ripple')
for i in range(6):
    C(S, '2.2u/100V', '+48V', 'GND', fp=FP_C1210)

# Bus voltage divider: 100k / 5.6k -> 62 V full-scale at 3.3 V
R(S, '100k', '+48V', 'VBUS_SENSE')
R(S, '5.6k', 'VBUS_SENSE', 'GND')
C(S, '10n', 'VBUS_SENSE', 'GND')

FP_FET = 'Package_TO_SOT_SMD:Infineon_PG-HSOF-8-1'
for ph in 'ABC':
    sw, phn = f'SW_{ph}', f'PHASE_{ph}'
    # gate driver
    part(S, 'U', 'Driver_FET:LM5109BMA', 'LM5109BMA', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
         {'VDD': '+12V', 'HI': f'PWM_{ph}H', 'LI': f'PWM_{ph}L', 'VSS': 'GND',
          'LO': f'LO_{ph}', 'HS': sw, 'HO': f'HO_{ph}', 'HB': f'HB_{ph}'})
    C(S, '1u/25V', '+12V', 'GND', fp=FP_C0805)
    R(S, '10k', f'PWM_{ph}H', 'GND')      # keep FETs off while MCU is in reset
    R(S, '10k', f'PWM_{ph}L', 'GND')
    D(S, 'ES1D', '+12V', f'HB_{ph}', fp=FP_SMA)
    C(S, '2.2u/25V', f'HB_{ph}', sw, fp=FP_C0805)
    # high side FET
    R(S, '4R7', f'HO_{ph}', f'GH_{ph}')      # ~100 ns edges: ~1.7 W switching loss/phase at 35 A
    R(S, '10k', f'GH_{ph}', sw)
    part(S, 'Q', 'Transistor_FET:IPT015N10N5', 'IPT015N10N5', FP_FET,
         {'G': f'GH_{ph}', 'D': '+48V', 'S': sw})
    # low side FET
    R(S, '4R7', f'LO_{ph}', f'GL_{ph}')
    R(S, '10k', f'GL_{ph}', 'GND')
    part(S, 'Q', 'Transistor_FET:IPT015N10N5', 'IPT015N10N5', FP_FET,
         {'G': f'GL_{ph}', 'D': sw, 'S': 'GND'})
    # 4-terminal (Kelvin) phase shunt: pins 1/4 carry current, 2/3 are the sense
    # taps, so copper drop in the pours never reaches the amplifier
    part(S, 'R', 'Device:R_Shunt', '0.5m', FP_SHUNT,
         {'1': sw, '4': phn, '2': f'ISP_{ph}', '3': f'ISN_{ph}'},
         MPN='Bourns CSS4J-4026R-L500F (LCSC C2076423) or Isabellenhuette BVR 4026; 0.5 mOhm 1% 5 W')
    # INA240A1: gain 20 -> +/-165 A full scale, so the 80 A hard trip is measurable
    part(S, 'U', 'Amplifier_Current:INA240A1D', 'INA240A1D', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
         {'+': f'ISP_{ph}', '-': f'ISN_{ph}', 'V+': '+3V3', 'GND': 'GND', 'REF1': '+3V3', 'REF2': 'GND',
          '5': f'ISO_{ph}'})
    C(S, '100n', '+3V3', 'GND')
    R(S, '100R', f'ISO_{ph}', f'ISENSE_{ph}')
    C(S, '1n', f'ISENSE_{ph}', 'GND')
    # motor phase output pad
    CONN(S, 1, [phn], fp=FP_TERMINAL, value=f'MOTOR_{ph}', MPN='M5 ring lug terminal')

# Board-mounted NTC next to low-side FETs
R(S, '10k', '+3V3', 'TEMP_FET')
part(S, 'TH', 'Device:Thermistor_NTC', '10k B3435', 'Resistor_SMD:R_0603_1608Metric',
     {'1': 'TEMP_FET', '2': 'GND'}, MPN='NCP18XH103F03RB')
C(S, '100n', 'TEMP_FET', 'GND')

# ============================================================================
# MCU SHEET
# ============================================================================
S = 'mcu'
part(S, 'U', 'MCU_ST_STM32G4:STM32G431CBTx', 'STM32G431CBT6', 'Package_QFP:LQFP-48_7x7mm_P0.5mm', {
    'VBAT': '+3V3', 'VDD': '+3V3', '24': '+3V3', '36': '+3V3', '48': '+3V3',
    'VDDA': '+3V3A', 'VREF+': '+3V3A', 'VSSA': 'GND', 'VSS': 'GND', '23': 'GND',
    'PG10': 'NRST',
    # TIM1 complementary PWM
    'PA8': 'PWM_AH', 'PA9': 'PWM_BH', 'PA10': 'PWM_CH',
    'PB13': 'PWM_AL', 'PB14': 'PWM_BL', 'PB15': 'PWM_CL',
    # ADC
    'PA0': 'ISENSE_A', 'PA1': 'ISENSE_B', 'PA2': 'ISENSE_C', 'PA3': 'VBUS_SENSE',
    'PA4': 'THROTTLE', 'PA5': 'TEMP_FET', 'PA6': 'TEMP_MOTOR',
    # Halls (EXTI10..12)
    'PB10': 'HALL_A', 'PB11': 'HALL_B', 'PB12': 'HALL_C',
    # UART (USART1)
    'PB6': 'UART_TX', 'PB7': 'UART_RX',
    # switches / power
    'PB3': 'PWR_BTN', 'PB4': 'DIR_SW', 'PB5': 'BRAKE', 'PB9': 'PWR_HOLD',
    'PB8': 'BOOT0', 'PC13': 'LED_STATUS', 'PB2': 'LED_FAULT',
    # SWD
    'PA13': 'SWDIO', 'PA14': 'SWCLK',
    # unused
    'PA7': None, 'PA11': None, 'PA12': None, 'PA15': None, 'PB0': None, 'PB1': None,
    'PC14': None, 'PC15': None, 'PF0': None, 'PF1': None,
})
for _ in range(4):
    C(S, '100n', '+3V3', 'GND')
C(S, '4.7u', '+3V3', 'GND', fp=FP_C0805)
part(S, 'FB', 'Device:FerriteBead_Small', '600R@100MHz', FP_FB, {'1': '+3V3', '2': '+3V3A'})
C(S, '1u', '+3V3A', 'GND')
C(S, '100n', '+3V3A', 'GND')
part(S, '#FLG', 'power:PWR_FLAG', 'PWR_FLAG', '', {'1': '+3V3A'})
C(S, '100n', 'NRST', 'GND')
R(S, '10k', 'BOOT0', 'GND')
R(S, '1k', 'LED_STATUS', 'LED_STATUS_A')
LED(S, 'GREEN', 'LED_STATUS_A', 'GND')
R(S, '1k', 'LED_FAULT', 'LED_FAULT_A')
LED(S, 'RED', 'LED_FAULT_A', 'GND')
CONN(S, 5, ['+3V3', 'SWDIO', 'SWCLK', 'NRST', 'GND'],
     fp='Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical', value='SWD')

# ============================================================================
# IO SHEET
# ============================================================================
S = 'io'
FP_ESD = 'Package_TO_SOT_SMD:SOT-23-6'

# Hall sensor + motor temperature connector (5V, GND, A, B, C, TEMP)
CONN(S, 6, ['+5V', 'GND', 'HALL_A_IN', 'HALL_B_IN', 'HALL_C_IN', 'MTEMP_IN'], value='HALL')
for h in 'ABC':
    R(S, '2.2k', '+3V3', f'HALL_{h}_IN')         # open-collector pull-up
    R(S, '1k', f'HALL_{h}_IN', f'HALL_{h}')
    C(S, '1n', f'HALL_{h}', 'GND')
R(S, '10k', '+3V3', 'MTEMP_IN')                   # motor NTC (10k) to GND
R(S, '1k', 'MTEMP_IN', 'TEMP_MOTOR')
C(S, '100n', 'TEMP_MOTOR', 'GND')
part(S, 'D', 'Power_Protection:ESDA6V1-5SC6', 'ESDA6V1-5SC6', FP_ESD,
     {'IO1': 'HALL_A_IN', 'IO2': 'HALL_B_IN', 'IO3': 'HALL_C_IN', 'IO4': 'MTEMP_IN',
      'IO5': 'THR_IN', 'GND': 'GND'})

# Throttle (hall throttle 0.8..4.2 V)  divider 12k/22k -> 2.72 V at 4.2 V
CONN(S, 3, ['+5V', 'THR_IN', 'GND'], value='THROTTLE')
R(S, '12k', 'THR_IN', 'THROTTLE')
R(S, '22k', 'THROTTLE', 'GND')
C(S, '100n', 'THROTTLE', 'GND')

# Direction switch (closed = reverse), brake switch (closed = brake)
CONN(S, 2, ['DIR_IN', 'GND'], value='FWD/REV')
R(S, '10k', '+3V3', 'DIR_SW')
R(S, '1k', 'DIR_IN', 'DIR_SW')
C(S, '100n', 'DIR_SW', 'GND')
CONN(S, 2, ['BRAKE_IN', 'GND'], value='BRAKE')
R(S, '10k', '+3V3', 'BRAKE')
R(S, '1k', 'BRAKE_IN', 'BRAKE')
C(S, '100n', 'BRAKE', 'GND')

# Momentary power button (to GND)
CONN(S, 2, ['PWR_SW', 'GND'], value='POWER_BTN')

# UART telemetry / command port (3.3 V logic)
CONN(S, 4, ['+3V3', 'UART_TX_C', 'UART_RX_C', 'GND'], value='UART')
R(S, '100R', 'UART_TX', 'UART_TX_C')
R(S, '100R', 'UART_RX_C', 'UART_RX')
R(S, '10k', '+3V3', 'UART_RX')

part(S, 'D', 'Power_Protection:ESDA6V1-5SC6', 'ESDA6V1-5SC6', FP_ESD,
     {'IO1': 'DIR_IN', 'IO2': 'BRAKE_IN', 'IO3': 'PWR_SW', 'IO4': 'UART_TX_C',
      'IO5': 'UART_RX_C', 'GND': 'GND'})

for _ in range(4):
    part(S, 'H', 'Mechanical:MountingHole', 'M3', 'MountingHole:MountingHole_3.2mm_M3', {})
