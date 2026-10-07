#!/usr/bin/env python3
"""Write hardware/bom.csv (grouped by value + footprint) from circuit.py."""
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import circuit  # noqa: E402

# Suggested orderable parts for the generic values (voltage/size matter here)
HINTS = {
    ('C', '2.2u/100V'): 'X7R 1210 100V (e.g. GRM32ER72A225KA35)',
    ('C', '100n/100V'): 'X7R 0805 100V',
    ('C', '220u/100V'): 'Low-ESR electrolytic 12.5mm, >=2.1A ripple each (Panasonic EEU-FS2A221)',
    ('C', '22u/25V'): 'X5R/X7R 1210 25V',
    ('C', '22u/10V'): 'X5R/X7R 1210 10V',
    ('C', '2.2u/25V'): 'X7R 0805 25V',
    ('C', '1u/25V'): 'X7R 0805 25V',
    ('C', '2.2n'): 'X7R 0603 50V (LM5164 BST)',
    ('R', '4R7'): '0603 1% (gate resistor)',
    ('D', 'SMCJ60CA'): 'TVS 60V standoff, 1500W, SMC (Littelfuse, LCSC C151290)',
    ('D', 'ES1D'): '200V 1A ultrafast, SMA (bootstrap)',
    ('D', '1N4148W'): '100V signal diode SOD-123',
    ('D', 'BAT46W'): '100V Schottky SOD-123',
    # specialised parts, with an LCSC / JLCPCB part number known to be stocked (Oct 2026)
    ('Q', 'IPT015N10N5'): 'Infineon IPT015N10N5ATMA1 (LCSC C108964)',
    ('U', 'LM5109BMA'): 'TI LM5109BMAX/NOPB (LCSC C116862)',
    ('U', 'LM5164DDA'): 'TI LM5164DDAR (LCSC C477928)',
    ('U', 'INA240A1D'): 'TI INA240A1DR (LCSC C2060769)',
    ('U', 'STM32G431CBT6'): 'ST STM32G431CBT6 (LCSC C529355)',
    ('U', 'AP2112K-3.3'): 'Diodes AP2112K-3.3TRG1 (LCSC C51118)',
    ('D', 'ESDA6V1-5SC6'): 'ST ESDA6V1-5SC6 (LCSC C6650)',
}


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'bom.csv')
    groups = {}
    for c in circuit.components:
        if c['ref'].startswith('#') or not c['footprint']:
            continue
        key = (c['value'], c['footprint'], c['fields'].get('MPN', ''))
        groups.setdefault(key, []).append(c['ref'])

    def sort_key(item):
        refs = item[1]
        prefix = ''.join(ch for ch in refs[0] if ch.isalpha())
        return (prefix, item[0][0])

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Qty', 'Value', 'Footprint', 'Part / notes', 'References'])
        for (value, fp, mpn), refs in sorted(groups.items(), key=sort_key):
            prefix = ''.join(ch for ch in refs[0] if ch.isalpha())
            note = mpn or HINTS.get((prefix, value), '')
            refs.sort(key=lambda r: (len(r), r))
            w.writerow([len(refs), value, fp.split(':')[1], note, ' '.join(refs)])
    print(f'wrote {out}: {sum(len(r) for r in groups.values())} parts, {len(groups)} lines')


if __name__ == '__main__':
    main()
