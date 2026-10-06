#!/usr/bin/env python3
"""
Independent connectivity check (KiCad 7's CLI has no ERC).

Exports the netlist with kicad-cli, then verifies that every pin of every
component landed on exactly the net circuit.py intended, and flags
single-node nets, unconnected pins and duplicate references.

usage: check_netlist.py path/to/bldc48.kicad_sch
"""
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import circuit  # noqa: E402
from gen_schematic import Libs, resolve_pins  # noqa: E402
from sexp import find, find1, parse  # noqa: E402


def main():
    sch = sys.argv[1]
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'n.net')
        subprocess.run(['kicad-cli', 'sch', 'export', 'netlist', '-o', out, sch],
                       check=True, capture_output=True)
        root = parse(open(out).read())

    actual = {}   # (ref, pin) -> net
    nets = {}
    for net in find(find1(root, 'nets'), 'net'):
        name = find1(net, 'name')[1].split('/')[-1]
        nodes = [(find1(nd, 'ref')[1], find1(nd, 'pin')[1]) for nd in find(net, 'node')]
        nets[name] = nodes
        for nd in nodes:
            actual[nd] = name

    refs = [find1(c, 'ref')[1] for c in find(find1(root, 'components'), 'comp')]
    errors = []
    dup = {r for r in refs if refs.count(r) > 1}
    if dup:
        errors.append(f'duplicate refs: {sorted(dup)}')

    libs = Libs('/usr/share/kicad/symbols')
    checked = 0
    for comp in circuit.components:
        if comp['ref'].startswith('#'):
            continue
        pins = libs.pins(comp['lib_id'])
        mapping = resolve_pins(comp, pins)
        for p in pins:
            num = p['number']
            want = mapping.get(num)
            if want is None and p['hidden'] and p['type'] == 'power_in':
                want = p['name']                 # implicit hidden power pin
            if want is None and p['hidden']:
                # stacked hidden pin: shares the net of the visible pin of same name
                twins = [q for q in pins if q['name'] == p['name'] and not q['hidden']]
                want = mapping.get(twins[0]['number']) if twins else None
            got = actual.get((comp['ref'], num))
            checked += 1
            if want is None:
                if got and not got.startswith('unconnected-'):
                    if len(nets.get(got, [])) > 1:
                        errors.append(f'{comp["ref"]}.{num} ({p["name"]}) should be NC, is on {got}')
                continue
            if got != want:
                errors.append(f'{comp["ref"]}.{num} ({p["name"]}): want {want}, got {got}')

    for name, nodes in nets.items():
        if len(nodes) == 1 and not name.startswith('unconnected-'):
            errors.append(f'net {name} has a single node {nodes}')

    print(f'{len(refs)} components, {len(nets)} nets, {checked} pins checked')
    for e in errors:
        print('ERROR', e)
    sys.exit(1 if errors else 0)


if __name__ == '__main__':
    main()
