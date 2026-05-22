#!/usr/bin/env python3
"""
BrickInterface KiCad Schematic Generator (CH552T version)

Generates BrickInterface.kicad_sch with all components placed and connected.
Run: python3 generate_brickinterface.py

The generated schematic uses simplified library symbol definitions.
After opening in KiCad, use "Tools > Update Symbols from Library" to get
full graphical representations from your KiCad standard library.
"""

import uuid
import math
import hashlib

uid = lambda: str(uuid.uuid4())

# Project namespace for deterministic UUID generation.
# This is a fixed UUID — do not change it, or all stable_uid() outputs will change.
_PROJECT_NS = uuid.UUID("a3b4c5d6-7890-4abc-def0-1234567890ab")

def stable_uid(key):
    """Generate a deterministic UUID for the given string key.

    Same input always produces the same UUID. Used for component UUIDs and
    pin UUIDs so that regenerating the schematic preserves identity, allowing
    KiCad's 'Update PCB from Schematic' to match by UUID without losing
    placement/routing work.
    """
    return str(uuid.uuid5(_PROJECT_NS, key))

G = 2.54  # Grid spacing

# ============================================================
# Pin S-expression helper
# ============================================================

def pin_s(num, name, etype, x, y, angle, length=2.54):
    return (
        f'        (pin {etype} line (at {x} {y} {angle}) (length {length})'
        f' (name "{name}" (effects (font (size 1.27 1.27))))'
        f' (number "{num}" (effects (font (size 1.27 1.27)))))'
    )

def prop_s(key, val, x, y, angle=0, hide=False):
    h = " hide" if hide else ""
    return f'      (property "{key}" "{val}" (at {x} {y} {angle}) (effects (font (size 1.27 1.27)){h}))'

# ============================================================
# Library symbol definitions
# ============================================================

SYM_DEFS = []
PIN_POS = {}

def def_resistor():
    k = "Device:R"
    PIN_POS["R"] = {"1": (0, 3.81), "2": (0, -3.81)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 0)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","R",2.032,0,90)}
{prop_s("Value","R",-2.032,0,90)}
{prop_s("Footprint","",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "R_0_1"
        (rectangle (start -1.016 -2.54) (end 1.016 2.54) (stroke (width 0.254) (type default)) (fill (type none)))
      )
      (symbol "R_1_1"
{pin_s("1","~","passive",0,3.81,270,1.27)}
{pin_s("2","~","passive",0,-3.81,90,1.27)}
      )
    )""")

def def_capacitor():
    k = "Device:C"
    PIN_POS["C"] = {"1": (0, 3.81), "2": (0, -3.81)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 0.254)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","C",1.524,0,90)}
{prop_s("Value","C",-1.524,0,90)}
{prop_s("Footprint","",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "C_0_1"
        (polyline (pts (xy -1.524 -0.508) (xy 1.524 -0.508)) (stroke (width 0.254) (type default)) (fill (type none)))
        (polyline (pts (xy -1.524 0.508) (xy 1.524 0.508)) (stroke (width 0.254) (type default)) (fill (type none)))
      )
      (symbol "C_1_1"
{pin_s("1","~","passive",0,3.81,270,3.302)}
{pin_s("2","~","passive",0,-3.81,90,3.302)}
      )
    )""")

def def_led():
    k = "Device:LED"
    PIN_POS["LED"] = {"1": (0, -3.81), "2": (0, 3.81)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","D",2.54,0,90)}
{prop_s("Value","LED",-2.54,0,90)}
{prop_s("Footprint","",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "LED_0_1"
        (polyline (pts (xy -1.27 1.27) (xy -1.27 -1.27)) (stroke (width 0.254) (type default)) (fill (type none)))
        (polyline (pts (xy -1.27 0) (xy 1.27 1.27) (xy 1.27 -1.27) (xy -1.27 0)) (stroke (width 0.254) (type default)) (fill (type none)))
      )
      (symbol "LED_1_1"
{pin_s("1","K","passive",0,-3.81,90,2.54)}
{pin_s("2","A","passive",0,3.81,270,2.54)}
      )
    )""")

def def_solder_jumper():
    k = "Jumper:SolderJumper_2_Open"
    PIN_POS["SJ"] = {"1": (-3.81, 0), "2": (3.81, 0)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 0)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","JP",0,-3.81,0)}
{prop_s("Value","SolderJumper_2_Open",0,3.81,0)}
{prop_s("Footprint","",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "SolderJumper_2_Open_0_1"
        (polyline (pts (xy -0.508 0.508) (xy 0.508 0.508) (xy 0.508 -0.508) (xy -0.508 -0.508) (xy -0.508 0.508)) (stroke (width 0) (type default)) (fill (type none)))
      )
      (symbol "SolderJumper_2_Open_1_1"
{pin_s("1","A","passive",-3.81,0,0,1.27)}
{pin_s("2","B","passive",3.81,0,180,1.27)}
      )
    )""")

def def_switch():
    k = "Switch:SW_Push"
    PIN_POS["SW"] = {"1": (-3.81, 0), "2": (3.81, 0)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 0)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","SW",1.27,2.54,0)}
{prop_s("Value","SW_Push",1.27,-2.54,0)}
{prop_s("Footprint","",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "SW_Push_0_1"
        (circle (center -2.032 0) (radius 0.508) (stroke (width 0) (type default)) (fill (type none)))
        (circle (center 2.032 0) (radius 0.508) (stroke (width 0) (type default)) (fill (type none)))
        (polyline (pts (xy 0 1.27) (xy 0 2.54)) (stroke (width 0) (type default)) (fill (type none)))
        (polyline (pts (xy -1.524 0.508) (xy 1.524 1.778)) (stroke (width 0) (type default)) (fill (type none)))
      )
      (symbol "SW_Push_1_1"
{pin_s("1","1","passive",-3.81,0,0,1.27)}
{pin_s("2","2","passive",3.81,0,180,1.27)}
      )
    )""")

def def_npn():
    k = "Transistor_BJT:MMBT2222A"
    PIN_POS["NPN"] = {"1": (-5.08, 0), "2": (2.54, 5.08), "3": (2.54, -5.08)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 0)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","Q",-5.08,-3.81,0)}
{prop_s("Value","MMBT2222A",-5.08,3.81,0)}
{prop_s("Footprint","Package_TO_SOT_SMD:SOT-23",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "MMBT2222A_0_1"
        (rectangle (start -1.27 -3.81) (end 1.27 3.81) (stroke (width 0.254) (type default)) (fill (type background)))
        (polyline (pts (xy 1.27 -1.27) (xy 2.54 -3.81)) (stroke (width 0.254) (type default)) (fill (type none)))
        (polyline (pts (xy 1.27 1.27) (xy 2.54 3.81)) (stroke (width 0.254) (type default)) (fill (type none)))
        (polyline (pts (xy -1.27 0) (xy -3.81 0)) (stroke (width 0.254) (type default)) (fill (type none)))
      )
      (symbol "MMBT2222A_1_1"
{pin_s("1","B","input",-5.08,0,0,1.27)}
{pin_s("2","E","passive",2.54,5.08,270,1.27)}
{pin_s("3","C","passive",2.54,-5.08,90,1.27)}
      )
    )""")

def def_usb_c():
    """USB-C receptacle USB 2.0. All 16 signal pads + 1 shield exposed
    so the cable works in both orientations (A and B rows must be tied
    in the schematic for redundant pins).
    """
    k = "Connector:USB_C_Receptacle_USB2.0"
    PIN_POS["USB_C"] = {
        # Top row (A)
        "A1":  (-7.62, -10.16),  # GND
        "A4":  (-7.62, -7.62),   # VBUS
        "A5":  (-7.62, -5.08),   # CC1
        "A6":  (-7.62, -2.54),   # D+
        "A7":  (-7.62, 0),       # D-
        "A8":  (-7.62, 2.54),    # SBU1 (NC for USB 2.0)
        "A9":  (-7.62, 5.08),    # VBUS
        "A12": (-7.62, 7.62),    # GND
        # Bottom row (B)
        "B1":  (-7.62, 10.16),   # GND
        "B4":  (-7.62, 12.7),    # VBUS
        "B5":  (-7.62, 15.24),   # CC2
        "B6":  (-7.62, 17.78),   # D+
        "B7":  (-7.62, 20.32),   # D-
        "B8":  (-7.62, 22.86),   # SBU2 (NC for USB 2.0)
        "B9":  (-7.62, 25.4),    # VBUS
        "B12": (-7.62, 27.94),   # GND
        # Shield
        "S1":  (-7.62, 30.48),   # SHIELD
    }
    pin_lines = [
        pin_s("A1",  "GND",    "power_in",      -7.62, -10.16, 0),
        pin_s("A4",  "VBUS",   "power_in",      -7.62, -7.62, 0),
        pin_s("A5",  "CC1",    "bidirectional", -7.62, -5.08, 0),
        pin_s("A6",  "D+",     "bidirectional", -7.62, -2.54, 0),
        pin_s("A7",  "D-",     "bidirectional", -7.62, 0, 0),
        pin_s("A8",  "SBU1",   "bidirectional", -7.62, 2.54, 0),
        pin_s("A9",  "VBUS",   "power_in",      -7.62, 5.08, 0),
        pin_s("A12", "GND",    "power_in",      -7.62, 7.62, 0),
        pin_s("B1",  "GND",    "power_in",      -7.62, 10.16, 0),
        pin_s("B4",  "VBUS",   "power_in",      -7.62, 12.7, 0),
        pin_s("B5",  "CC2",    "bidirectional", -7.62, 15.24, 0),
        pin_s("B6",  "D+",     "bidirectional", -7.62, 17.78, 0),
        pin_s("B7",  "D-",     "bidirectional", -7.62, 20.32, 0),
        pin_s("B8",  "SBU2",   "bidirectional", -7.62, 22.86, 0),
        pin_s("B9",  "VBUS",   "power_in",      -7.62, 25.4, 0),
        pin_s("B12", "GND",    "power_in",      -7.62, 27.94, 0),
        pin_s("S1",  "SHIELD", "passive",       -7.62, 30.48, 0),
    ]
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","J",0,-13,0)}
{prop_s("Value","USB_C_USB2.0",0,33,0)}
{prop_s("Footprint","Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "USB_C_Receptacle_USB2.0_0_1"
        (rectangle (start -5.08 -11.43) (end 5.08 31.75) (stroke (width 0.254) (type default)) (fill (type background)))
      )
      (symbol "USB_C_Receptacle_USB2.0_1_1"
{chr(10).join(pin_lines)}
      )
    )""")

def def_conn_2x10():
    k = "Connector_Generic:Conn_02x10_Odd_Even"
    pins = {}
    for row in range(10):
        y = -11.43 + row * 2.54
        odd = str(2 * row + 1)
        even = str(2 * row + 2)
        pins[odd] = (-3.81, y)
        pins[even] = (3.81, y)
    PIN_POS["Conn2x10"] = pins
    pin_lines = []
    for row in range(10):
        y = -11.43 + row * 2.54
        odd = 2 * row + 1
        even = 2 * row + 2
        pin_lines.append(pin_s(str(odd), f"Pin_{odd}", "passive", -3.81, y, 0, 1.27))
        pin_lines.append(pin_s(str(even), f"Pin_{even}", "passive", 3.81, y, 180, 1.27))
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","J",0,-13.97,0)}
{prop_s("Value","Conn_02x10",0,13.97,0)}
{prop_s("Footprint","Connector_IDC:IDC-Header_2x10_P2.54mm_Vertical",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "Conn_02x10_Odd_Even_0_1"
        (rectangle (start -2.54 -12.7) (end 2.54 12.7) (stroke (width 0.254) (type default)) (fill (type background)))
      )
      (symbol "Conn_02x10_Odd_Even_1_1"
{chr(10).join(pin_lines)}
      )
    )""")

def def_ch552t():
    """CH552T TSSOP-20 custom symbol."""
    k = "MCU_WCH:CH552T"
    # Pin layout: left side pins 1-10, right side pins 11-20
    # Left side (pins 1-10): pin at x=-12.7, spaced 2.54 apart
    # Right side (pins 11-20): pin at x=12.7, spaced 2.54 apart
    # Pin map per official WCH CH552 datasheet (CH552DS1.PDF, page 3) and
    # cross-checked against WeAct Studio CH55x core board V1.0 schematic.
    mcu_pins = [
        # Left side (pins 1-10, top to bottom)
        ("1",  "P3.2",  "bidirectional", -12.7, -11.43, 0),
        ("2",  "P1.4",  "bidirectional", -12.7,  -8.89, 0),
        ("3",  "P1.5",  "bidirectional", -12.7,  -6.35, 0),
        ("4",  "P1.6",  "bidirectional", -12.7,  -3.81, 0),
        ("5",  "P1.7",  "bidirectional", -12.7,  -1.27, 0),
        ("6",  "RST",   "input",         -12.7,   1.27, 0),
        ("7",  "P1.0",  "bidirectional", -12.7,   3.81, 0),
        ("8",  "P1.1",  "bidirectional", -12.7,   6.35, 0),
        ("9",  "P3.1",  "bidirectional", -12.7,   8.89, 0),
        ("10", "P3.0",  "bidirectional", -12.7,  11.43, 0),
        # Right side (pins 11-20, bottom to top)
        ("11", "P3.3",  "bidirectional", 12.7,   11.43, 180),
        ("12", "P3.4",  "bidirectional", 12.7,    8.89, 180),
        ("13", "P3.5",  "bidirectional", 12.7,    6.35, 180),
        ("14", "P3.6",  "bidirectional", 12.7,    3.81, 180),
        ("15", "P3.7",  "bidirectional", 12.7,    1.27, 180),
        ("16", "P1.3",  "bidirectional", 12.7,   -1.27, 180),
        ("17", "P1.2",  "bidirectional", 12.7,   -3.81, 180),
        ("18", "GND",   "power_in",      12.7,   -6.35, 180),
        ("19", "VCC",   "power_in",      12.7,   -8.89, 180),
        ("20", "V33",   "power_out",     12.7,  -11.43, 180),
    ]
    PIN_POS["CH552T"] = {p[0]: (p[3], p[4]) for p in mcu_pins}
    pin_lines = [pin_s(p[0], p[1], p[2], p[3], p[4], p[5]) for p in mcu_pins]
    SYM_DEFS.append(f"""\
    (symbol "{k}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)
{prop_s("Reference","U",0,-15.24,0)}
{prop_s("Value","CH552T",0,15.24,0)}
{prop_s("Footprint","Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm",0,0,0,True)}
{prop_s("Datasheet","~",0,0,0,True)}
      (symbol "CH552T_0_1"
        (rectangle (start -10.16 -12.7) (end 10.16 12.7) (stroke (width 0.254) (type default)) (fill (type background)))
      )
      (symbol "CH552T_1_1"
{chr(10).join(pin_lines)}
      )
    )""")

def def_power_5v():
    k = "power:+5V"
    PIN_POS["+5V"] = {"1": (0, 0)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (power) (pin_names (offset 0)) (exclude_from_sim no) (in_bom no) (on_board yes)
      (property "Reference" "#PWR" (at 0 -3.81 0) (effects (font (size 1.27 1.27)) hide))
      (property "Value" "+5V" (at 0 3.81 0) (effects (font (size 1.27 1.27))))
      (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
      (symbol "+5V_0_1"
        (polyline (pts (xy -0.762 1.27) (xy 0 2.54)) (stroke (width 0) (type default)) (fill (type none)))
        (polyline (pts (xy 0 0) (xy 0 1.27)) (stroke (width 0) (type default)) (fill (type none)))
        (polyline (pts (xy 0 2.54) (xy 0.762 1.27)) (stroke (width 0) (type default)) (fill (type none)))
      )
      (symbol "+5V_1_1"
        (pin power_in line (at 0 0 90) (length 0) (name "+5V" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
      )
    )""")

def def_gnd():
    k = "power:GND"
    PIN_POS["GND"] = {"1": (0, 0)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (power) (pin_names (offset 0)) (exclude_from_sim no) (in_bom no) (on_board yes)
      (property "Reference" "#PWR" (at 0 3.81 0) (effects (font (size 1.27 1.27)) hide))
      (property "Value" "GND" (at 0 -3.81 0) (effects (font (size 1.27 1.27))))
      (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
      (symbol "GND_0_1"
        (polyline (pts (xy 0 0) (xy 0 -1.27) (xy 1.27 -1.27) (xy 0 -2.54) (xy -1.27 -1.27) (xy 0 -1.27)) (stroke (width 0) (type default)) (fill (type none)))
      )
      (symbol "GND_1_1"
        (pin power_in line (at 0 0 270) (length 0) (name "GND" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
      )
    )""")

def def_pwr_flag():
    k = "power:PWR_FLAG"
    PIN_POS["PWR_FLAG"] = {"1": (0, 0)}
    SYM_DEFS.append(f"""\
    (symbol "{k}" (power) (pin_names (offset 0)) (exclude_from_sim no) (in_bom no) (on_board yes)
      (property "Reference" "#FLG" (at 0 2.032 0) (effects (font (size 1.27 1.27)) hide))
      (property "Value" "PWR_FLAG" (at 0 4.064 0) (effects (font (size 1.27 1.27))))
      (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
      (symbol "PWR_FLAG_0_1"
        (polyline (pts (xy 0 0) (xy 0 1.524) (xy -1.016 2.286) (xy 0 3.048) (xy 1.016 2.286) (xy 0 1.524)) (stroke (width 0) (type default)) (fill (type none)))
      )
      (symbol "PWR_FLAG_1_1"
        (pin power_out line (at 0 0 90) (length 0) (name "pwr_flag" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))
      )
    )""")

# ============================================================
# Component placement
# ============================================================

components = []
pwr_counter = [0]

def add(sym_key, lib_id, ref, value, footprint, x, y, angle=0):
    comp = {
        "sym_key": sym_key, "lib_id": lib_id,
        "ref": ref, "value": value, "footprint": footprint,
        "x": x, "y": y, "angle": angle,
        "uuid": stable_uid(f"comp:{ref}"),
        "pin_uuids": {pn: stable_uid(f"pin:{ref}:{pn}") for pn in PIN_POS[sym_key]},
    }
    components.append(comp)
    return comp

def add_pwr(sym_key, lib_id, x, y, angle=0):
    pwr_counter[0] += 1
    ref = f"#PWR{pwr_counter[0]:03d}"
    val = "+5V" if "5V" in lib_id else "GND"
    return add(sym_key, lib_id, ref, val, "", x, y, angle)

flg_counter = [0]
def add_pwr_flag(net_name, x, y):
    """Place a PWR_FLAG symbol at (x,y) and label it with the net name."""
    flg_counter[0] += 1
    ref = f"#FLG{flg_counter[0]:03d}"
    comp = add("PWR_FLAG", "power:PWR_FLAG", ref, "PWR_FLAG", "", x, y, 0)
    # Label the net at the same point so PWR_FLAG is on that net
    label(net_name, x, y, 0)
    return comp

def pw(comp, pin_num):
    """World position of a pin connection point.

    KiCad uses Y-up in symbol coordinates (positive Y = up on screen) but
    Y-down in schematic world coordinates. We invert Y when converting from
    symbol space to world space.
    """
    px, py = PIN_POS[comp["sym_key"]][pin_num]
    a = math.radians(comp["angle"])
    c, s = math.cos(a), math.sin(a)
    # Apply rotation in symbol coords (Y-up math convention)
    rx = px * c - py * s
    ry = px * s + py * c
    # Convert to world coords (Y-down)
    wx = comp["x"] + rx
    wy = comp["y"] - ry
    return (round(wx, 2), round(wy, 2))

# ============================================================
# Wires, labels, no-connects
# ============================================================

wires = []
labels = []
no_connects = []

def wire(x1, y1, x2, y2):
    wires.append((x1, y1, x2, y2))

def label(name, x, y, angle=0):
    labels.append((name, x, y, angle))

def label_at(name, comp, pin_num, angle=0):
    x, y = pw(comp, pin_num)
    label(name, x, y, angle)

def wire_label(name, comp, pin_num, dx, dy, label_angle=0):
    x, y = pw(comp, pin_num)
    wire(x, y, x + dx, y + dy)
    label(name, x + dx, y + dy, label_angle)

def pwr_at(sym_key, lib_id, comp, pin_num, angle=0):
    x, y = pw(comp, pin_num)
    return add_pwr(sym_key, lib_id, x, y, angle)

def nc(comp, pin_num):
    x, y = pw(comp, pin_num)
    no_connects.append((x, y))

# ============================================================
# Build schematic
# ============================================================

def build():
    # ---- Define all library symbols ----
    def_resistor()
    def_capacitor()
    def_led()
    def_solder_jumper()
    def_switch()
    def_npn()
    def_usb_c()
    def_conn_2x10()
    def_ch552t()
    def_power_5v()
    def_gnd()
    def_pwr_flag()

    # ========================================
    # Place components  (A4 sheet: ~297 x 210 mm)
    # ========================================

    # -- MCU (center-right area) --
    U1 = add("CH552T", "MCU_WCH:CH552T", "U1",
             "CH552T", "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm", 130, 100)

    # -- USB-C connector (far left) --
    J1 = add("USB_C", "Connector:USB_C_Receptacle_USB2.0", "J1",
             "USB_C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", 35, 80)

    # -- CC pulldown resistors --
    R1 = add("R", "Device:R", "R1", "5.1k",
             "Resistor_SMD:R_0603_1608Metric", 52, 100)
    R2 = add("R", "Device:R", "R2", "5.1k",
             "Resistor_SMD:R_0603_1608Metric", 60, 100)

    # -- Decoupling caps --
    C1 = add("C", "Device:C", "C1", "100nF",
             "Capacitor_SMD:C_0603_1608Metric", 155, 60)
    C2 = add("C", "Device:C", "C2", "10uF",
             "Capacitor_SMD:C_0805_2012Metric", 163, 60)
    C3 = add("C", "Device:C", "C3", "100nF",
             "Capacitor_SMD:C_0603_1608Metric", 171, 60)

    # -- Interface A IDC header (right side) --
    J2 = add("Conn2x10", "Connector_Generic:Conn_02x10_Odd_Even", "J2",
             "Interface_A", "Connector_IDC:IDC-Header_2x10_P2.54mm_Vertical", 220, 110)

    # Ribbon pins 1/3 are hardwired to +5V (powers Interface A optocoupler
    # LEDs on the computer side — required for any signal to pass).

    # -- IR transmitter section (bottom) --
    # Two 5mm THT IR LEDs, horizontal-mount — lens points out the edge of the
    # board. TSAL6200 (medium ±17°) + TSAL6100 (narrow ±10°) per AnalysIR
    # recommendation: combined narrow-throw + wider fill for max range.
    D1 = add("LED", "Device:LED", "D1", "TSAL6200",
             "LED_THT:LED_D5.0mm_Horizontal_O1.27mm_Z3.0mm_IRGrey", 105, 170)
    D2 = add("LED", "Device:LED", "D2", "TSAL6100",
             "LED_THT:LED_D5.0mm_Horizontal_O1.27mm_Z3.0mm_IRGrey", 120, 170)

    # Constant-current driver: Q1 drives, Q2 senses Vbe across R4.
    # I_LED_total = ~0.7V / R4 = 210mA (105mA per LED, within DC continuous spec).
    Q1 = add("NPN", "Transistor_BJT:MMBT2222A", "Q1",
             "MMBT2222A", "Package_TO_SOT_SMD:SOT-23", 108, 200)
    Q2 = add("NPN", "Transistor_BJT:MMBT2222A", "Q2",
             "MMBT2222A", "Package_TO_SOT_SMD:SOT-23", 130, 200)

    # Q1 base pullup from CH552 GPIO (CH552 supplies enable current; Q2 steals
    # it when current exceeds set point).
    R3 = add("R", "Device:R", "R3", "470R",
             "Resistor_SMD:R_0603_1608Metric", 90, 195, 90)

    # R_sense — sets total LED current. 1206 size for power dissipation
    # headroom: I^2*R = 0.21^2 * 3.3 = 145mW peak, well under 1206's 250mW rating.
    R4 = add("R", "Device:R", "R4", "3R3",
             "Resistor_SMD:R_1206_3216Metric", 119, 215)

    # -- Indicator LEDs (bottom-left) --
    D4 = add("LED", "Device:LED", "D4", "Green",
             "LED_SMD:LED_0603_1608Metric", 50, 170)
    D5 = add("LED", "Device:LED", "D5", "Amber",
             "LED_SMD:LED_0603_1608Metric", 65, 170)
    R7 = add("R", "Device:R", "R7", "1k",
             "Resistor_SMD:R_0603_1608Metric", 50, 157)
    R8 = add("R", "Device:R", "R8", "1k",
             "Resistor_SMD:R_0603_1608Metric", 65, 157)

    # -- Bootloader entry button --
    # Hold during USB plug-in to force CH552 into ROM bootloader.
    # +5V -> SW1 -> R9 (10k current limit) -> USB_DP (P3.6).
    # The 10k limits short-circuit current if user presses during normal USB op.
    SW1 = add("SW", "Switch:SW_Push", "SW1", "BOOT",
              "BrickInterface:SW_TS3315A", 35, 105)
    R9  = add("R",  "Device:R",       "R9",  "10k",
              "Resistor_SMD:R_0603_1608Metric", 50, 105)

    # ========================================
    # Net connections
    # ========================================

    # --- USB VBUS direct to +5V (no polyfuse on this simpler board) ---
    # All 4 VBUS pads (A4, A9, B4, B9) tied to +5V for cable-orientation independence
    pwr_at("+5V", "power:+5V", J1, "A4")
    pwr_at("+5V", "power:+5V", J1, "A9")
    pwr_at("+5V", "power:+5V", J1, "B4")
    pwr_at("+5V", "power:+5V", J1, "B9")

    # --- USB GND — all 4 GND pads + shield ---
    pwr_at("GND", "power:GND", J1, "A1")
    pwr_at("GND", "power:GND", J1, "A12")
    pwr_at("GND", "power:GND", J1, "B1")
    pwr_at("GND", "power:GND", J1, "B12")
    pwr_at("GND", "power:GND", J1, "S1")  # Shield to GND

    # --- USB SBU1/SBU2 unused for USB 2.0 ---
    nc(J1, "A8")
    nc(J1, "B8")

    # --- PWR_FLAG markers — declare +5V and GND as externally sourced ---
    add_pwr_flag("+5V", 25, 70)
    add_pwr_flag("GND", 25, 95)

    # --- CC1, CC2 pulldowns ---
    wire_label("CC1", J1, "A5", 5, 0, 0)
    wire_label("CC1", R1, "1", 0, -5, 90)
    pwr_at("GND", "power:GND", R1, "2")

    wire_label("CC2", J1, "B5", 5, 0, 0)
    wire_label("CC2", R2, "1", 0, -5, 90)
    pwr_at("GND", "power:GND", R2, "2")

    # --- USB D+/D- — both A and B row pins tied for cable orientation ---
    wire_label("USB_DP", J1, "A6", 5, 0, 0)
    wire_label("USB_DP", J1, "B6", 5, 0, 0)
    wire_label("USB_DP", U1, "14", 5, 0, 0)   # P3.6 (UDP) — chip pin 14
    wire_label("USB_DP", R9, "2", 5, 0, 0)    # Bootloader pull via R9

    # Bootloader button: +5V -> SW1 -> BOOT_PULL -> R9 -> USB_DP
    pwr_at("+5V", "power:+5V", SW1, "1")
    wire_label("BOOT_PULL", SW1, "2", 5, 0, 0)
    wire_label("BOOT_PULL", R9, "1", -5, 0, 180)

    wire_label("USB_DM", J1, "A7", 5, 0, 0)
    wire_label("USB_DM", J1, "B7", 5, 0, 0)
    wire_label("USB_DM", U1, "15", 5, 0, 0)   # P3.7 (UDM) — chip pin 15

    # --- MCU power ---
    pwr_at("+5V", "power:+5V", U1, "19")      # VCC — chip pin 19
    pwr_at("GND", "power:GND", U1, "18")      # GND — chip pin 18

    # --- V33 bypass cap ---
    wire_label("V33", U1, "20", 5, 0, 0)      # V33 — chip pin 20
    wire_label("V33", C3, "1", 0, -5, 90)
    pwr_at("GND", "power:GND", C3, "2")

    # --- Decoupling caps on +5V ---
    pwr_at("+5V", "power:+5V", C1, "1")
    pwr_at("GND", "power:GND", C1, "2")
    pwr_at("+5V", "power:+5V", C2, "1")
    pwr_at("GND", "power:GND", C2, "2")

    # --- Interface A outputs (MCU -> IDC header) ---
    # Sequential pin mapping for clean PCB routing — IDC pins map to
    # adjacent MCU pins on the same edge, no trace crossings.
    wire_label("IFACE_OUT0", U1, "2", -5, 0, 180)   # P1.4
    wire_label("IFACE_OUT0", J2, "6", 5, 0, 0)

    wire_label("IFACE_OUT1", U1, "3", -5, 0, 180)   # P1.5
    wire_label("IFACE_OUT1", J2, "8", 5, 0, 0)

    wire_label("IFACE_OUT2", U1, "4", -5, 0, 180)   # P1.6
    wire_label("IFACE_OUT2", J2, "10", 5, 0, 0)

    wire_label("IFACE_OUT3", U1, "5", -5, 0, 180)   # P1.7
    wire_label("IFACE_OUT3", J2, "12", 5, 0, 0)

    wire_label("IFACE_OUT4", U1, "9", -5, 0, 180)   # P3.1 — chip pin 9
    wire_label("IFACE_OUT4", J2, "14", 5, 0, 0)

    wire_label("IFACE_OUT5", U1, "10", -5, 0, 180)  # P3.0 — chip pin 10
    wire_label("IFACE_OUT5", J2, "16", 5, 0, 0)

    # Interface A inputs
    wire_label("IFACE_IN6", U1, "8", -5, 0, 180)    # P1.1 — chip pin 8
    wire_label("IFACE_IN6", J2, "18", 5, 0, 0)

    wire_label("IFACE_IN7", U1, "7", -5, 0, 180)    # P1.0 — chip pin 7
    wire_label("IFACE_IN7", J2, "20", 5, 0, 0)

    # IDC header GND pins (odd pins 5-19)
    for p in ["5", "7", "9", "11", "13", "15", "17", "19"]:
        pwr_at("GND", "power:GND", J2, p)

    # IDC header +5V (pins 1, 3) — required by Interface A optocouplers
    pwr_at("+5V", "power:+5V", J2, "1")
    pwr_at("+5V", "power:+5V", J2, "3")

    # IDC header pins 2, 4 unused
    nc(J2, "2")
    nc(J2, "4")

    # --- IR LED drive (constant-current source) ---
    # CH552 P3.4 -> R3 (pullup) -> Q1 base. Q2 collector also pulled to Q1 base
    # (steals base current to regulate Q1's emitter current at 0.7V/R4).
    wire_label("IR_DRIVE", U1, "12", 5, 0, 0)        # P3.4 / PWM2 — chip pin 12
    wire_label("IR_DRIVE", R3, "1", -5, 0, 180)      # Pullup top

    wire_label("IR_BASE", R3, "2", 5, 0, 0)          # Pullup bottom
    wire_label("IR_BASE", Q1, "1", -5, 0, 180)       # Q1 base
    wire_label("IR_BASE", Q2, "3", 0, -5, 90)        # Q2 collector (steal)

    # LED anodes to +5V, cathodes tied together to Q1 collector
    pwr_at("+5V", "power:+5V", D1, "2")
    pwr_at("+5V", "power:+5V", D2, "2")
    wire_label("IR_COLL", D1, "1", 0, 5, 270)
    wire_label("IR_COLL", D2, "1", 0, 5, 270)
    wire_label("IR_COLL", Q1, "3", 0, -5, 90)        # Q1 collector

    # Q1 emitter -> R4 (R_sense) -> GND, also feeds Q2 base for sensing
    wire_label("IR_SENSE", Q1, "2", 5, 0, 0)         # Q1 emitter
    wire_label("IR_SENSE", Q2, "1", -5, 0, 180)      # Q2 base
    wire_label("IR_SENSE", R4, "1", 0, -5, 90)       # R_sense top
    pwr_at("GND", "power:GND", R4, "2")              # R_sense bottom
    pwr_at("GND", "power:GND", Q2, "2")              # Q2 emitter

    # --- Indicator LEDs ---
    # Power LED: +5V -> R7 -> D4 -> GND (always on)
    pwr_at("+5V", "power:+5V", R7, "1")
    wire_label("LED_PWR", R7, "2", 0, 5, 270)
    wire_label("LED_PWR", D4, "2", 0, -5, 90)
    pwr_at("GND", "power:GND", D4, "1")

    # Activity LED: P3.2 -> R8 -> D5 -> GND
    wire_label("ACT_DRIVE", U1, "1", -5, 0, 180)    # P3.2
    wire_label("ACT_DRIVE", R8, "1", 0, -5, 90)
    wire_label("LED_ACT", R8, "2", 0, 5, 270)
    wire_label("LED_ACT", D5, "2", 0, -5, 90)
    pwr_at("GND", "power:GND", D5, "1")

    # --- Unused MCU pins: no-connect ---
    nc(U1, "6")   # RST (internal pull-up; reset only via download button)
    nc(U1, "11")  # P3.3 (unused — power LED is now always-on from +5V)
    nc(U1, "13")  # P3.5 (unused)
    nc(U1, "16")  # P1.3 / XO (no external crystal)
    nc(U1, "17")  # P1.2 / XI (no external crystal)

# ============================================================
# Output generator
# ============================================================

SHEET_UUID = stable_uid("sheet:root")  # Root schematic sheet UUID — stable across regenerations

def generate_instance(comp):
    lines = []
    lines.append(
        f'  (symbol (lib_id "{comp["lib_id"]}") (at {comp["x"]} {comp["y"]} {comp["angle"]})'
        f' (unit 1) (exclude_from_sim no) (in_bom {"no" if comp["ref"].startswith("#") else "yes"})'
        f' (on_board yes) (dnp no)'
        f' (uuid "{comp["uuid"]}")'
    )
    lines.append(f'    (property "Reference" "{comp["ref"]}" (at {comp["x"]+2} {comp["y"]-2} 0)'
                 f' (effects (font (size 1.27 1.27)){"" if not comp["ref"].startswith("#") else " hide"}))')
    lines.append(f'    (property "Value" "{comp["value"]}" (at {comp["x"]+2} {comp["y"]+2} 0)'
                 f' (effects (font (size 1.27 1.27))))')
    lines.append(f'    (property "Footprint" "{comp["footprint"]}" (at {comp["x"]} {comp["y"]} 0)'
                 f' (effects (font (size 1.27 1.27)) hide))')
    lines.append(f'    (property "Datasheet" "~" (at {comp["x"]} {comp["y"]} 0)'
                 f' (effects (font (size 1.27 1.27)) hide))')
    for pn, pu in comp["pin_uuids"].items():
        lines.append(f'    (pin "{pn}" (uuid "{pu}"))')
    # Instances block — required by KiCad 7+ for proper annotation
    lines.append(f'    (instances')
    lines.append(f'      (project ""')
    lines.append(f'        (path "/{SHEET_UUID}"')
    lines.append(f'          (reference "{comp["ref"]}") (unit 1)')
    lines.append(f'        )')
    lines.append(f'      )')
    lines.append(f'    )')
    lines.append(f'  )')
    return "\n".join(lines)

def generate_wire(w):
    u = stable_uid(f"wire:{w[0]}:{w[1]}:{w[2]}:{w[3]}")
    return (f'  (wire (pts (xy {w[0]} {w[1]}) (xy {w[2]} {w[3]}))'
            f' (stroke (width 0) (type default)) (uuid "{u}"))')

def generate_label(l):
    u = stable_uid(f"label:{l[0]}:{l[1]}:{l[2]}:{l[3]}")
    return (f'  (label "{l[0]}" (at {l[1]} {l[2]} {l[3]})'
            f' (effects (font (size 1.27 1.27))) (uuid "{u}"))')

def generate_no_connect(nc_pos):
    u = stable_uid(f"nc:{nc_pos[0]}:{nc_pos[1]}")
    return f'  (no_connect (at {nc_pos[0]} {nc_pos[1]}) (uuid "{u}"))'

def generate_schematic():
    build()

    out = []
    out.append(f'(kicad_sch')
    out.append(f'  (version 20231120)')
    out.append(f'  (generator "BrickInterface_CH552T_generator")')
    out.append(f'  (generator_version "1.0")')
    out.append(f'  (uuid "{SHEET_UUID}")')
    out.append(f'  (paper "A4")')
    out.append(f'')
    out.append(f'  (lib_symbols')
    for sd in SYM_DEFS:
        out.append(sd)
    out.append(f'  )')
    out.append(f'')

    for comp in components:
        out.append(generate_instance(comp))

    for w in wires:
        out.append(generate_wire(w))

    for l in labels:
        out.append(generate_label(l))

    for nc_pos in no_connects:
        out.append(generate_no_connect(nc_pos))

    out.append(f')')
    return "\n".join(out)

# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    import os
    content = generate_schematic()
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "BrickInterface.kicad_sch")
    with open(out_path, "w") as f:
        f.write(content)
    print(f"Generated: {out_path}")
    print(f"  {len(components)} components")
    print(f"  {len(wires)} wires")
    print(f"  {len(labels)} labels")
    print(f"  {len(no_connects)} no-connects")
    print()
    print()
    print("Open in KiCad. DO NOT use 'Tools > Update Symbols from Library' —")
    print("it replaces our embedded simplified symbols with standard library")
    print("versions that have different pin positions, breaking all wiring.")
    print("The lib_symbol_mismatch warnings in ERC are cosmetic only.")
