"""
Netlist-level description of the 48 V / 35 A hub-motor controller.

Every component is described once here.  gen_schematic.py turns this into a
hierarchical KiCad 7 schematic (symbols pulled straight from the stock KiCad
libraries so the pin-outs are the library's, not hand-drawn), and gen_pcb.py
turns the same data into a placed PCB with nets assigned.

Pin keys can be the pin *number* (str) or the pin *name* as it appears in the
KiCad library symbol.  A net name of None marks the pin as no-connect.
Nets whose names match a power symbol (GND, +48V, +5V, +3V3) are drawn
with power symbols; everything else gets a net label.
"""

POWER_NETS = {'GND', '+48V', '+5V', '+3V3'}

SHEETS = [
    ('power', 'Power input, soft-latch, 5 V buck, 3.3 V LDO'),
    ('bridge', 'DRV8353RS gate driver, 3-phase power stage, low-side current sensing'),
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
# Power wires: 12 AWG soldered into 2.8 mm plated holes (strain-relieve the leads)
FP_WIRE = 'bldc48:SolderWire_12AWG_D2.8mm_OD5.4mm'
FP_SHUNT = 'Resistor_SMD:R_Shunt_Isabellenhuette_BVR4026'
FP_FB = 'Inductor_SMD:L_0603_1608Metric'


def jst_gh(n):
    return f'Connector_JST:JST_GH_BM{n:02d}B-GHS-TBT_1x{n:02d}-1MP_P1.25mm_Vertical'


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
    if fp is None:
        kw.setdefault('MPN', f'JST BM{n:02d}B-GHS-TBT (1.25 mm, latching)')
    return part(sheet, 'J', lib, value or f'Conn_01x{n:02d}', fp or jst_gh(n),
                {str(i + 1): net for i, net in enumerate(nets)}, **kw)


# ============================================================================
# POWER SHEET
# ============================================================================
S = 'power'

# Battery input: 12 AWG wires soldered straight into plated holes (strain-relieve
# the leads; put an XT90-S anti-spark plug and a 40 A fuse in the battery lead)
CONN(S, 1, ['+48V'], fp=FP_WIRE, value='BAT+', MPN='12 AWG wire, soldered')
CONN(S, 1, ['GND'], fp=FP_WIRE, value='BAT-', MPN='12 AWG wire, soldered')
part(S, 'D', 'Device:D_TVS', 'SMCJ60CA', FP_SMC, {'1': '+48V', '2': 'GND'})

# --- Soft power latch ---------------------------------------------------------
# Momentary switch pulls PWR_SW to GND (switch only ever sees <3.3 V) -> current
# flows out of Q1 base through R(47k) + D -> Q1 (high-voltage PNP) turns on ->
# EN_RAW feeds the DRV8353 buck's RT/SD pin, which shuts the buck down when it
# is pulled low -> 5 V and 3.3 V come up -> MCU boots and drives PWR_HOLD high
# (Q2 keeps Q1 on).  To switch off the MCU sees a long press on PWR_BTN, waits
# for release, drops PWR_HOLD.
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

# --- 5 V buck inside the DRV8353RS (LM5008A core, 6-95 V in, 350 mA) ----------
# COT: t_on = 1.25e-10 * R_T / V_in -> 150k: 390 ns at 48 V -> f = 5/(48*390n) = 270 kHz
# (R_T hangs off EN_RAW, so the latch also shuts the buck down: RT/SD < 0.7 V)
# V_out = 2.5 V * (1 + 4.02k/4.02k) = 5.0 V; type-3 ripple injection as in the
# DRV8353RS-EVM; non-synchronous, so a 100 V Schottky catches the inductor current
R(S, '150k', 'EN_RAW', 'RT_SD')
R(S, '100k', 'RCL', 'GND')
C(S, '1u/100V', '+48V', 'GND', fp=FP_C0805)
C(S, '10n', 'BST5', 'SW5')
C(S, '1u', 'VCC_B', 'GND')
D(S, 'SS110', 'GND', 'SW5', fp=FP_SMA, MPN='100 V 1 A Schottky, SMA (e.g. MDD SS110)')
part(S, 'L', 'Device:L', '100u', 'Inductor_SMD:L_Bourns_SRN6045TA',
     {'1': 'SW5', '2': '+5V'}, MPN='Bourns SRN6045TA-101M or eq. (100 uH, Isat >= 0.5 A, 6x6 mm)')
R(S, '4.02k', '+5V', 'FB5')
R(S, '4.02k', 'FB5', 'GND')
R(S, '8.2k', 'SW5', 'RIP5')
C(S, '47n', 'RIP5', '+5V')
C(S, '330p', 'RIP5', 'FB5')
C(S, '22u/10V', '+5V', 'GND', fp=FP_C1210)
C(S, '4.7u', '+5V', 'GND', fp=FP_C0805)

# --- 3.3 V LDO ---------------------------------------------------------------
part(S, 'U', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3', 'Package_TO_SOT_SMD:SOT-23-5',
     {'VIN': '+5V', 'EN': '+5V', 'GND': 'GND', 'VOUT': '+3V3'})
C(S, '1u', '+5V', 'GND')
C(S, '4.7u', '+3V3', 'GND', fp=FP_C0805)
R(S, '2.2k', '+3V3', 'PWR_LED_A')
LED(S, 'GREEN', 'PWR_LED_A', 'GND')

# Power flags (tell ERC these nets are driven)
for net in ['+48V', 'GND', '+5V']:
    part(S, '#FLG', 'power:PWR_FLAG', 'PWR_FLAG', '', {'1': net})

# ============================================================================
# BRIDGE SHEET
# ============================================================================
S = 'bridge'

# DC-link capacitance: 3 x 1000 uF/100 V standing beside the bridge (~17 A rms
# ripple at 35 A battery current, short peaks to ~27 A) + 6 x 2.2 uF/100 V X7R
# at the low-side FET sources
for i in range(3):
    CP(S, '1000u/100V', '+48V', 'GND', fp='Capacitor_THT:CP_Radial_D18.0mm_P7.50mm',
       MPN='Aishi ERS1KM102M35OT (LCSC C724666), 18x35 mm, 7.5 mm pitch, 105 C 10000 h')
for i in range(6):
    C(S, '2.2u/100V', '+48V', 'GND', fp=FP_C1210)

# Bus voltage divider: 100k / 5.6k -> 62 V full-scale at 3.3 V
R(S, '100k', '+48V', 'VBUS_SENSE')
R(S, '5.6k', 'VBUS_SENSE', 'GND')
C(S, '10n', 'VBUS_SENSE', 'GND')

# --- DRV8353RS: 100 V three-phase smart gate driver -------------------------
# Gate current is set over SPI (IDRIVE), so no gate resistors; the charge pump
# and the VGLS regulator run the gates straight off VM (no 12 V rail).  The three
# current-sense amplifiers read the low-side shunts (gain 20 over SPI).
part(S, 'U', 'bldc48:DRV8353RS', 'DRV8353RS', 'bldc48:TI-RGZ-48-QFN', {
    'GND': 'GND', 'AGND': 'GND', 'DGND': 'GND', 'PAD': 'GND',
    'VM': '+48V', 'VDRAIN': '+48V', 'VIN': '+48V',
    'VGLS': 'VGLS', 'CPL': 'CPL', 'CPH': 'CPH', 'VCP': 'VCP', 'DVDD': 'DVDD',
    'GHA': 'GH_A', 'SHA': 'SW_A', 'GLA': 'GL_A', 'SPA': 'SP_A', 'SNA': 'SN_A',
    'GHB': 'GH_B', 'SHB': 'SW_B', 'GLB': 'GL_B', 'SPB': 'SP_B', 'SNB': 'SN_B',
    'GHC': 'GH_C', 'SHC': 'SW_C', 'GLC': 'GL_C', 'SPC': 'SP_C', 'SNC': 'SN_C',
    'SOA': 'SO_A', 'SOB': 'SO_B', 'SOC': 'SO_C', 'VREF': '+3V3',
    'nFAULT': 'DRV_FAULT', 'SDO': 'DRV_SDO', 'SDI': 'DRV_SDI', 'SCLK': 'DRV_SCK',
    'nSCS': 'DRV_CS', 'ENABLE': 'DRV_EN',
    'INHA': 'PWM_AH', 'INLA': 'PWM_AL', 'INHB': 'PWM_BH', 'INLB': 'PWM_BL',
    'INHC': 'PWM_CH', 'INLC': 'PWM_CL',
    'SW': 'SW5', 'VCC': 'VCC_B', 'BST': 'BST5', 'RCL': 'RCL', 'RT/SD': 'RT_SD', 'FB': 'FB5',
}, MPN='TI DRV8353RSRGZR (LCSC C506246)')
C(S, '100n/100V', '+48V', 'GND', fp=FP_C0805)          # VM
C(S, '2.2u/100V', '+48V', 'GND', fp=FP_C1210)          # VM
C(S, '47n/100V', 'CPL', 'CPH', fp=FP_C0805)            # charge-pump flying cap (sees VM)
C(S, '1u/16V', 'VCP', '+48V')                          # VCP to VDRAIN
C(S, '1u/16V', 'VGLS', 'GND')
C(S, '1u', 'DVDD', 'GND')
C(S, '100n', '+3V3', 'GND')                            # VREF
R(S, '10k', '+3V3', 'DRV_FAULT')                       # open-drain nFAULT
R(S, '4.7k', '+3V3', 'DRV_SDO')                        # open-drain SDO
R(S, '10k', '+3V3', 'DRV_CS')                          # deselected while the MCU boots
R(S, '100k', 'DRV_EN', 'GND')                          # driver asleep until the MCU wakes it

FP_FET = 'Package_TO_SOT_SMD:Infineon_PG-HSOF-8-1'
for ph in 'ABC':
    sw = f'SW_{ph}'
    part(S, 'Q', 'Transistor_FET:IPT015N10N5', 'IPT015N10N5', FP_FET,
         {'G': f'GH_{ph}', 'D': '+48V', 'S': sw})
    part(S, 'Q', 'Transistor_FET:IPT015N10N5', 'IPT015N10N5', FP_FET,
         {'G': f'GL_{ph}', 'D': sw, 'S': f'LS_{ph}'})
    # low-side 4-terminal (Kelvin) shunt: pins 1/4 carry current (FET source ->
    # GND), 2/3 are the sense taps to SPx / SNx, so pour drop never reaches the CSA
    part(S, 'R', 'Device:R_Shunt', '0.5m', FP_SHUNT,
         {'1': f'LS_{ph}', '4': 'GND', '2': f'SP_{ph}', '3': f'SN_{ph}'},
         MPN='Bourns CSS4J-4026R-L500F (LCSC C2076423), 0.5 mOhm 1% 5 W')
    C(S, '1n', f'SP_{ph}', f'SN_{ph}')
    R(S, '100R', f'SO_{ph}', f'ISENSE_{ph}')
    C(S, '1n', f'ISENSE_{ph}', 'GND')
    # motor phase wire: the switch node itself
    CONN(S, 1, [sw], fp=FP_WIRE, value=f'MOTOR_{ph}', MPN='12 AWG wire, soldered')

# Board-mounted NTC next to the low-side FETs
R(S, '10k', '+3V3', 'TEMP_FET')
part(S, 'TH', 'Device:Thermistor_NTC', '10k B3435', 'Resistor_SMD:R_0603_1608Metric',
     {'1': 'TEMP_FET', '2': 'GND'}, MPN='Murata NCP18XH103F03RB (LCSC C13564)')
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
    # DRV8353RS: SPI1 (AF5) + enable / fault
    'PB3': 'DRV_SCK', 'PB4': 'DRV_SDO', 'PB5': 'DRV_SDI', 'PA15': 'DRV_CS',
    'PB0': 'DRV_EN', 'PB1': 'DRV_FAULT',
    # switches / power
    'PA11': 'PWR_BTN', 'PA12': 'DIR_SW', 'PC14': 'BRAKE', 'PB9': 'PWR_HOLD',
    'PB8': 'BOOT0', 'PC13': 'LED_STATUS', 'PB2': 'LED_FAULT',
    # SWD
    'PA13': 'SWDIO', 'PA14': 'SWCLK',
    # unused
    'PA7': None, 'PC15': None, 'PF0': None, 'PF1': None,
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
CONN(S, 5, ['+3V3', 'SWDIO', 'SWCLK', 'NRST', 'GND'], value='SWD')

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
