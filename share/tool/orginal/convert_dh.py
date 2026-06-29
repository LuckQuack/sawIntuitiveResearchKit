import sys
import os
import re
import math

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_original_file(filepath):
    """Parses variables from an original-4xxxxx file.

    Returns a dict with keys:
      coupling       : {1..4: [float x4]}
      upper_limit    : [float x4] or None
      lower_limit    : [float x4] or None
      dh_params      : {1..6: [float x7]} or {}  (may be absent)
      tip_length     : float or None
      max_close_torque: float or None
      max_open_torque : float or None
    """
    with open(filepath, 'r') as f:
        content = f.read()

    params = {
        'coupling': {},
        'upper_limit': None,
        'lower_limit': None,
        'dh_params': {},
        'tip_length': None,
        'max_close_torque': None,
        'max_open_torque': None,
    }

    for idx, val_str in re.findall(
            r'data\.coupling\(INDEX(\d),:\)\s*=\s*\[\s*([^\]]+)\s*\]', content):
        params['coupling'][int(idx)] = [float(x) for x in val_str.split()]

    m = re.search(r'data\.joint\.signal_range\(UPPER_LIMIT,:\)\s*=\s*\[\s*([^\]]+)\s*\]', content)
    if m:
        params['upper_limit'] = [float(x) for x in m.group(1).split()]

    m = re.search(r'data\.joint\.signal_range\(LOWER_LIMIT,:\)\s*=\s*\[\s*([^\]]+)\s*\]', content)
    if m:
        params['lower_limit'] = [float(x) for x in m.group(1).split()]

    for idx, val_str in re.findall(
            r'data\.dh_params\(INDEX(\d),:\)\s*=\s*\[\s*([^\]]+)\s*\]', content):
        params['dh_params'][int(idx)] = [float(x) for x in val_str.split()]

    m = re.search(r'data\.tip_length\s*=\s*\[\s*([^\]]+)\s*\]', content)
    if m:
        params['tip_length'] = float(m.group(1).strip())

    m = re.search(r'data\.grip\.max_close_torque\s*=\s*\[\s*([^\]]+)\s*\]', content)
    if m:
        params['max_close_torque'] = float(m.group(1).strip())

    m = re.search(r'data\.grip\.max_open_torque\s*=\s*\[\s*([^\]]+)\s*\]', content)
    if m:
        params['max_open_torque'] = float(m.group(1).strip())

    return params


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def fmt_deg_comment(rad_val):
    deg = round(rad_val * 180.0 / math.pi)
    return f"{deg} degrees"

def fmt_rad(val):
    """Format a radian value to a compact string matching dVRK JSON conventions."""
    aval = abs(val)
    known = [
        (4.45058959,  "4.4506"),
        (4.53785606,  "4.53786"),
        (1.22173048,  "1.2217"),
        (1.39626340,  "1.39626"),
        (1.48352986,  "1.4835"),
        (1.27409035,  "1.27409"),
        (1.57079633,  "1.5708"),
        (0.78539816,  "0.785398"),
        (0.73303829,  "0.733038"),
        (0.34906585,  "0.349066"),
        (0.69813170,  "0.698132"),
        (0.52359878,  "0.5236"),
        (0.08726646,  "0.087266"),
        (0.17453293,  "0.174533"),
        (0.26179939,  "0.261799"),
        (1.04719755,  "1.0472"),
    ]
    for ref, s in known:
        if abs(aval - ref) < 1e-5:
            return f"-{s}" if val < 0 else s
    # Generic fallback
    s = f"{aval:.6f}".rstrip('0').rstrip('.')
    return f"-{s}" if val < 0 else (s or "0.0")


# ---------------------------------------------------------------------------
# DH column indices in data.dh_params row:
#   [type, A, sin(alpha), cos(alpha), D, sin(theta), cos(theta)]
#
# alpha  = sin(alpha) * pi/2   (valid only when sin(alpha) in {-1, 0, 1})
# offset = sin(theta) * pi/2   (same assumption)
# ---------------------------------------------------------------------------
HALF_PI = math.pi / 2.0

def dh_alpha(row):
    return row[2] * HALF_PI          # sin(alpha) * pi/2

def dh_offset(row):
    return row[5] * HALF_PI          # sin(theta) * pi/2

def dh_A(row):
    return row[1]

def dh_D(row):
    return row[4]


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------

def convert_to_json_content(inst_id, params):
    """Generate JSON string from parsed parameters.

    Only uses values that are directly present in the original file.
    Raises ValueError if required data (dh_params) is absent.
    """
    dh = params['dh_params']
    coupling = params['coupling']
    upper = params['upper_limit']
    lower = params['lower_limit']
    max_close = params['max_close_torque'] or 0.0

    is_spatula_hook = (max_close == 0.0)

    # ---- DH joints --------------------------------------------------------
    # All three active joints must have dh_params; abort if missing.
    for idx, name in [(1, 'roll'), (2, 'wrist_pitch'), (3, 'wrist_yaw')]:
        if idx not in dh:
            raise ValueError(
                f"  ERROR: data.dh_params(INDEX{idx},:) absent in original — "
                f"cannot generate DH for joint '{name}' without heuristics.")

    row1, row2, row3 = dh[1], dh[2], dh[3]

    d1      = dh_D(row1)
    alpha1  = dh_alpha(row1)   # should be 0
    a1      = dh_A(row1)       # should be 0
    offset1 = dh_offset(row1)  # should be 0

    d2      = dh_D(row2)       # should be 0
    alpha2  = dh_alpha(row2)   # should be -pi/2
    a2      = dh_A(row2)       # should be 0
    offset2 = dh_offset(row2)  # should be -pi/2

    d3      = dh_D(row3)       # should be 0
    alpha3  = dh_alpha(row3)   # should be -pi/2
    a3      = dh_A(row3)
    offset3 = dh_offset(row3)  # should be -pi/2

    # ---- Joint limits (from signal_range) ---------------------------------
    if lower and upper:
        qmin1, qmax1 = lower[0], upper[0]
        qmin2, qmax2 = lower[1], upper[1]
        qmin3, qmax3 = lower[2], upper[2]
    else:
        raise ValueError(
            "  ERROR: data.joint.signal_range absent in original — "
            "cannot generate joint limits without heuristics.")

    # ---- Jaw --------------------------------------------------------------
    if is_spatula_hook:
        # Special dVRK software workaround: qmax is 0.5 (not 0) to avoid
        # warnings; this is a documented software constraint, not ISI data.
        jaw_str = (
            '// spatula doesn\'t have jaws\n'
            '        "qmin":  0.0, // 0 degrees\n'
            '        "qmax":  0.5, // 0 degrees, but not exactly to avoid warnings\n'
            '        "ftmax": 0.0'
        )
    else:
        jaw_qmin = lower[3] if lower else None
        jaw_qmax = upper[3] if upper else None
        jaw_ftmax = max_close

        if jaw_qmin is None or jaw_qmax is None:
            raise ValueError(
                "  ERROR: signal_range has fewer than 4 elements — "
                "cannot generate jaw limits.")

        jaw_str = (
            f'"qmin": {fmt_rad(jaw_qmin)}, // {fmt_deg_comment(jaw_qmin)}\n'
            f'        "qmax":  {fmt_rad(jaw_qmax)},  //  {fmt_deg_comment(jaw_qmax)}\n'
            f'        "ftmax": {jaw_ftmax}'
        )

    # ---- Coupling matrix --------------------------------------------------
    if is_spatula_hook:
        # dVRK hack: last disk is removed, coupling rows 3 and 4 are replaced.
        c11 = coupling[1][0] if 1 in coupling else None
        c22 = coupling[2][1] if 2 in coupling else None
        c32 = coupling[3][1] if 3 in coupling else None
        c43 = coupling[4][2] if 4 in coupling else None
        if any(v is None for v in [c11, c22, c32, c43]):
            raise ValueError("  ERROR: coupling rows missing for spatula/hook.")
        c_rows = [
            [c11, 0.0, 0.0, 0.0],
            [0.0, c22, 0.0, 0.0],
            [0.0, c32, -c43, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        c_decimals = 4
        c_comment = (
            '// -- proper matrix (last 2 actuators share last joint)\n'
            '        // "ActuatorToJointPosition" : [[ -1.5632,  0.0000,  0.0000,  0.0000],\n'
            '        //                              [  0.0000,  1.1353,  0.0000,  0.0000],\n'
            '        //                              [  0.0000,  0.7827, -0.5291, -0.5291],\n'
            '        //                              [  0.0000,  0.0000,  1.0583,  1.0583]]\n'
            '        // -- dVRK hack: last disk removed, disk 6 coupling doubled\n'
            '        // note that we don\'t use the last actuator\n'
            '        '
        )
        coupling_appendix = ""
    else:
        c_rows = [coupling.get(r, [0.0, 0.0, 0.0, 0.0]) for r in range(1, 5)]
        c_decimals = 6
        c_comment = ""
        coupling_appendix = ", ISI numbers rounded to 6 digits"

    # Format coupling rows
    formatted_c = []
    for i, row in enumerate(c_rows):
        parts = []
        for x in row:
            if c_decimals == 4 and abs(abs(x) - 1.05829511) < 1e-4:
                parts.append("-1.0582" if x < 0 else " 1.0582")
            else:
                parts.append(f"{x: .{c_decimals}f}")
        row_str = ", ".join(parts)
        if i == 3:
            if is_spatula_hook:
                formatted_c.append(f"[ {row_str}]] // note that we don't use the last actuator")
            else:
                formatted_c.append(f"[ {row_str}]]")
        else:
            formatted_c.append(f"[ {row_str}],")

    # ---- Assemble template ------------------------------------------------
    template = f"""/* -*- Mode: Javascript; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
{{
    // see dVRK user guide
    "DH": {{
        "convention": "modified",
        "joints": [
            {{
                "name": "roll",
                "alpha":  {alpha1:.4f}, "A":  {a1:.4f}, "theta":  0.0000, "D":  {d1:.4f},
                "type": "revolute",
                "mode": "active",
                "offset":  {offset1:.4f},
                "qmin": {fmt_rad(qmin1)}, // {fmt_deg_comment(qmin1)}
                "qmax":  {fmt_rad(qmax1)}, //  {fmt_deg_comment(qmax1)}
                "ftmax": 0.33
            }},
            {{
                "name": "wrist_pitch",
                "alpha": {alpha2:.4f}, "A":  {a2:.4f}, "theta":  0.0000, "D":  {d2:.4f},
                "type": "revolute",
                "mode": "active",
                "offset": {offset2:.4f},
                "qmin": {fmt_rad(qmin2)}, // {fmt_deg_comment(qmin2)}
                "qmax":  {fmt_rad(qmax2)}, //  {fmt_deg_comment(qmax2)}
                "ftmax": 0.25
            }},
            {{
                "name": "wrist_yaw",
                "alpha": {alpha3:.4f}, "A":  {a3:.4f}, "theta":  0.0000, "D":  {d3:.4f},
                "type": "revolute",
                "mode": "active",
                "offset": {offset3:.4f},
                "qmin": {fmt_rad(qmin3)}, // {fmt_deg_comment(qmin3)}
                "qmax":  {fmt_rad(qmax3)}, //  {fmt_deg_comment(qmax3)}
                "ftmax": 0.20
            }}
        ]
    }}
    ,
    "jaw" : {{
        {jaw_str}
    }}
    ,
    // rotation to match ISI convention (for read-only research API on commercial da Vinci)
    "tooltip_offset" : [[ 0.0, -1.0,  0.0,  0.0],
                        [ 0.0,  0.0,  1.0,  0.0],
                        [-1.0,  0.0,  0.0,  0.0],
                        [ 0.0,  0.0,  0.0,  1.0]]
    ,
    // values from the dVRK user guide, see tool appendix C{coupling_appendix}
    "coupling" : {{
        {c_comment}"ActuatorToJointPosition" : [{formatted_c[0]}
                                     {formatted_c[1]}
                                     {formatted_c[2]}
                                     {formatted_c[3]}
    }}
}}
"""
    return template


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 convert_dh.py <input_path> [<output_dir>]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else '.'

    if os.path.isdir(input_path):
        files = sorted(
            os.path.join(input_path, f)
            for f in os.listdir(input_path)
            if f.startswith('original-')
        )
    else:
        files = [input_path]

    tool_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tool')

    for f in files:
        m = re.search(r'original-(\d{6})', os.path.basename(f))
        if not m:
            print(f"Skipping {f} (no 6-digit ID found)")
            continue

        inst_id = m.group(1)
        params = parse_original_file(f)

        try:
            json_content = convert_to_json_content(inst_id, params)
        except ValueError as e:
            print(f"Skipping original-{inst_id}: {e}")
            continue

        # Preserve existing tool filename if available
        out_name = f"tool-{inst_id}.json"
        if os.path.isdir(tool_dir):
            for tf in os.listdir(tool_dir):
                if inst_id in tf and tf.endswith('.json'):
                    out_name = tf
                    break

        out_path = os.path.join(output_dir, out_name)
        os.makedirs(output_dir, exist_ok=True)
        with open(out_path, 'w') as out_f:
            out_f.write(json_content)
        print(f"Converted original-{inst_id} -> {out_name}")


if __name__ == '__main__':
    main()
