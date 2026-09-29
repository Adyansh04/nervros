#!/usr/bin/env python3
"""Dumps what ROS 2's own interface parser sees, as JSON, for the golden tests.

The Rust parser must agree with `rosidl_adapter.parser` on every field, type, array shape,
default, constant, comment and unit. This script runs the reference parser and writes what it
found; the Rust tests compare their own result with that file.

    source /opt/ros/jazzy/setup.bash
    python3 tools/dump_golden.py tests/fixtures/std_msgs > tests/golden/std_msgs.json
    python3 tools/dump_golden.py --scan /opt/ros/jazzy/share > all.json

A PACKAGE_DIR holds msg/, srv/ and action/ folders and is named like the package. --scan takes
a folder of such directories, for example an install's share/ folder; folders without
interfaces are skipped.

Output: {"<package>": {"<msg|srv|action>/<Name>": {...}}}, one interface per line. Types keep the
reference parser's fields (`pkg`, `name`, `bound`, `array`, `size`, `upper`); comments are the
reference parser's lists of lines. JSON has no NaN or infinity, so a non-finite float is written
as {"nonfinite": "nan"} (or "inf", "-inf").

With --lenient a file the reference parser rejects is recorded as {"kind": ..., "error": "<the
exception>"} instead of stopping the run, which lets the tests fuzz both parsers with bad input.
"""

import argparse
import glob
import json
import math
import os
import sys


def import_parser(ros_prefix):
    try:
        import rosidl_adapter.parser as parser
    except ImportError:
        for path in glob.glob(os.path.join(ros_prefix, 'lib', 'python3*', 'site-packages')):
            sys.path.insert(0, path)
        import rosidl_adapter.parser as parser
    return parser


def value(v):
    if isinstance(v, list):
        return [value(x) for x in v]
    if isinstance(v, float) and not math.isfinite(v):
        return {'nonfinite': repr(v)}
    return v


def type_(t):
    return {'pkg': t.pkg_name, 'name': t.type, 'bound': t.string_upper_bound,
            'array': t.is_array, 'size': t.array_size, 'upper': t.is_upper_bound}


def annotated(element, out):
    out['comment'] = list(element.annotations.get('comment', []))
    out['unit'] = element.annotations.get('unit')
    return out


def message(spec):
    fields = []
    for f in spec.fields:
        entry = {'name': f.name, 'type': type_(f.type)}
        if f.default_value is not None:
            entry['default'] = value(f.default_value)
        fields.append(annotated(f, entry))
    constants = [annotated(c, {'name': c.name, 'type': c.type, 'value': value(c.value)})
                 for c in spec.constants]
    return {'comment': list(spec.annotations.get('comment', [])), 'fields': fields,
            'constants': constants}


def interface(parser, kind, package, path):
    if kind == 'msg':
        return {'kind': kind, 'message': message(parser.parse_message_file(package, path))}
    if kind == 'srv':
        spec = parser.parse_service_file(package, path)
        return {'kind': kind, 'request': message(spec.request), 'response': message(spec.response)}
    spec = parser.parse_action_file(package, path)
    return {'kind': kind, 'goal': message(spec.goal), 'result': message(spec.result),
            'feedback': message(spec.feedback)}


def package(parser, directory, lenient):
    name = os.path.basename(os.path.normpath(directory))
    found = {}
    for kind in ('msg', 'srv', 'action'):
        for path in sorted(glob.glob(os.path.join(directory, kind, f'*.{kind}'))):
            stem = os.path.splitext(os.path.basename(path))[0]
            try:
                found[f'{kind}/{stem}'] = interface(parser, kind, name, path)
            except Exception as e:  # the reference raises many kinds of exception
                if not lenient:
                    raise
                found[f'{kind}/{stem}'] = {'kind': kind, 'error': type(e).__name__}
    return name, found


def main():
    args = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    args.add_argument('dirs', nargs='*', metavar='PACKAGE_DIR')
    args.add_argument('--scan', action='append', default=[], metavar='DIR')
    args.add_argument('--ros-prefix', default='/opt/ros/jazzy')
    args.add_argument('--lenient', action='store_true', help='record rejected files, do not stop')
    args.add_argument('-o', '--output', help='write here instead of stdout')
    opts = args.parse_args()

    parser = import_parser(opts.ros_prefix)
    dirs = list(opts.dirs)
    for scan in opts.scan:
        dirs += sorted(os.path.join(scan, d) for d in os.listdir(scan))
    packages = {}
    for d in dirs:
        if os.path.isdir(d):
            name, found = package(parser, d, opts.lenient)
            if found:
                packages[name] = found

    lines = []
    for name, found in packages.items():
        body = ',\n'.join(
            f'{json.dumps(key)}:{json.dumps(v, separators=(",", ":"), ensure_ascii=False)}'
            for key, v in found.items())
        lines.append(f'{json.dumps(name)}:{{\n{body}\n}}')
    text = '{\n' + ',\n'.join(lines) + '\n}\n'
    if opts.output:
        with open(opts.output, 'w', encoding='utf-8') as f:
            f.write(text)
    else:
        sys.stdout.write(text)


if __name__ == '__main__':
    main()
