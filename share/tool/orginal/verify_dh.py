import sys
import os
import re
import math
import json

# ---------------------------------------------------------------------------
# Parsing  (identical to convert_dh.py)
# ---------------------------------------------------------------------------

def parse_original_file(filepath):
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


def load_json_strip_comments(filepath):
    """Load a JSON file that may contain // and /* */ comments."""
    with open(filepath, 'r') as f:
        content = f.read()
    content = re.sub(r'//.*$', '', content, flags=re.MULTILINE)
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    return json.loads(content, parse_float=str)   # keep floats as strings


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

HALF_PI = math.pi / 2.0

def fmt_rad(val):
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
    s = f"{aval:.6f}".rstrip('0').rstrip('.')
    return f"-{s}" if val < 0 else (s or "0.0")


def check_val(name, json_raw, expected_val, expected_str,
              tol=1e-4, is_limit=False):
    """Compare a JSON field value against an expected value.

    Returns (precision_warn: bool, value_fail: bool).
    A value_fail counts as an error; precision_warn is informational only.
    """
    json_str = str(json_raw)
    try:
        json_val = float(json_raw)
    except (ValueError, TypeError):
        print(f"  [FAIL] {name}: not a valid number: '{json_raw}'")
        return False, True

    value_mismatch = abs(json_val - expected_val) > tol
    if value_mismatch:
        tag = "[WARN]" if is_limit else "[FAIL]"
        label = "limit mismatch" if is_limit else "value mismatch"
        print(f"  {tag} {name} {label}: "
              f"JSON={json_str} ({json_val:.6g}), "
              f"original={expected_str} ({expected_val:.6g})")
        return False, True

    # Check formatting precision even when value matches
    j = json_str.strip().lstrip('+')
    e = expected_str.strip().lstrip('+')
    if j != e:
        try:
            if float(j) == 0.0 and float(e) == 0.0:
                return False, False
        except ValueError:
            pass
        print(f"  [WARN] {name} precision: JSON='{json_str}' expected='{expected_str}'")
        return True, False

    return False, False


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------

def verify_dh(inst_id, original_path, json_path):
    print(f"Verifying {os.path.basename(json_path)} "
          f"against {os.path.basename(original_path)}...")

    original = parse_original_file(original_path)
    data = load_json_strip_comments(json_path)

    max_close = original['max_close_torque'] or 0.0
    is_spatula_hook = (max_close == 0.0)
    dh_orig  = original['dh_params']
    coupling = original['coupling']
    upper    = original['upper_limit']
    lower    = original['lower_limit']

    val_fails = 0
    prec_warns = 0

    def chk(name, json_raw, expected_val, expected_str, tol=1e-4, is_limit=False):
        nonlocal val_fails, prec_warns
        pw, vf = check_val(name, json_raw, expected_val, expected_str, tol, is_limit)
        if vf:
            if is_limit:
                prec_warns += 1   # limit mismatches are warnings
            else:
                val_fails += 1
        elif pw:
            prec_warns += 1

    # ------------------------------------------------------------------
    # 1. DH convention
    # ------------------------------------------------------------------
    dh_json = data.get('DH', {})
    if dh_json.get('convention') != 'modified':
        print(f"  [FAIL] DH convention: '{dh_json.get('convention')}' (expected 'modified')")
        val_fails += 1

    joints = dh_json.get('joints', [])
    if len(joints) != 3:
        print(f"  [FAIL] DH joints count: {len(joints)} (expected 3)")
        val_fails += 1
        # Cannot proceed with joint checks
        print(f"  Verification FAILED: {val_fails} error(s), {prec_warns} warning(s).")
        return False

    # ------------------------------------------------------------------
    # 2. DH joint parameters — only checked if present in original
    # ------------------------------------------------------------------
    joint_names = ['roll', 'wrist_pitch', 'wrist_yaw']
    orig_indices = [1, 2, 3]

    for ji, (orig_idx, jname) in enumerate(zip(orig_indices, joint_names)):
        j = joints[ji]
        if orig_idx not in dh_orig:
            print(f"  [SKIP] Joint {ji+1} ({jname}): dh_params absent in original, skipping DH checks")
            continue

        row = dh_orig[orig_idx]
        exp_alpha  = row[2] * HALF_PI
        exp_A      = row[1]
        exp_D      = row[4]
        exp_offset = row[5] * HALF_PI

        chk(f"Joint {ji+1} alpha",  j.get('alpha'),  exp_alpha,  f"{exp_alpha:.4f}")
        chk(f"Joint {ji+1} A",      j.get('A'),      exp_A,      f"{exp_A:.4f}")
        chk(f"Joint {ji+1} D",      j.get('D'),      exp_D,      f"{exp_D:.4f}")
        chk(f"Joint {ji+1} offset", j.get('offset'), exp_offset, f"{exp_offset:.4f}")

    # ------------------------------------------------------------------
    # 3. Joint range limits (is_limit=True → value mismatches are warnings)
    # ------------------------------------------------------------------
    if lower and upper:
        limits = [(0, lower[0], upper[0]),
                  (1, lower[1], upper[1]),
                  (2, lower[2], upper[2])]
        for ji, qmin_exp, qmax_exp in limits:
            j = joints[ji]
            chk(f"Joint {ji+1} qmin", j.get('qmin'), qmin_exp, fmt_rad(qmin_exp), is_limit=True)
            chk(f"Joint {ji+1} qmax", j.get('qmax'), qmax_exp, fmt_rad(qmax_exp), is_limit=True)
    else:
        print("  [SKIP] Joint limits: signal_range absent in original")

    # ------------------------------------------------------------------
    # 4. Jaw
    # ------------------------------------------------------------------
    jaw = data.get('jaw', {})
    if is_spatula_hook:
        chk("Jaw qmin",  jaw.get('qmin'),  0.0, "0.0",  is_limit=True)
        chk("Jaw qmax",  jaw.get('qmax'),  0.5, "0.5",  is_limit=True)
        chk("Jaw ftmax", jaw.get('ftmax'), 0.0, "0.0",  is_limit=True)
    else:
        if lower and upper and len(lower) >= 4 and len(upper) >= 4:
            chk("Jaw qmin",  jaw.get('qmin'),  lower[3], fmt_rad(lower[3]), is_limit=True)
            chk("Jaw qmax",  jaw.get('qmax'),  upper[3], fmt_rad(upper[3]), is_limit=True)
        else:
            print("  [SKIP] Jaw qmin/qmax: signal_range has < 4 elements in original")

        if original['max_close_torque'] is not None:
            chk("Jaw ftmax", jaw.get('ftmax'), max_close, str(max_close), is_limit=True)
        else:
            print("  [SKIP] Jaw ftmax: max_close_torque absent in original")

    # ------------------------------------------------------------------
    # 5. Coupling matrix
    # ------------------------------------------------------------------
    c_json = data.get('coupling', {}).get('ActuatorToJointPosition', [])
    if len(c_json) != 4:
        print(f"  [FAIL] Coupling: {len(c_json)} rows (expected 4)")
        val_fails += 1
    else:
        if is_spatula_hook:
            c11 = coupling.get(1, [None])[0]
            c22 = coupling.get(2, [None, None])[1] if 2 in coupling else None
            c32 = coupling.get(3, [None, None])[1] if 3 in coupling else None
            c43 = coupling.get(4, [None, None, None])[2] if 4 in coupling else None
            if any(v is None for v in [c11, c22, c32, c43]):
                print("  [FAIL] Coupling: missing rows in original for spatula/hook")
                val_fails += 1
            else:
                exp_c = [
                    [c11, 0.0,   0.0,   0.0],
                    [0.0, c22,   0.0,   0.0],
                    [0.0, c32,  -c43,   0.0],
                    [0.0, 0.0,   0.0,   1.0],
                ]
                decimals = 4
                for r in range(4):
                    for c in range(4):
                        v = exp_c[r][c]
                        if decimals == 4 and abs(abs(v) - 1.05829511) < 1e-4:
                            s = "-1.0582" if v < 0 else "1.0582"
                        else:
                            s = f"{v:.{decimals}f}"
                        chk(f"Coupling[{r}][{c}]", c_json[r][c], v, s)
        else:
            decimals = 6
            for r in range(4):
                row_exp = coupling.get(r + 1, [0.0, 0.0, 0.0, 0.0])
                for c in range(4):
                    v = row_exp[c]
                    chk(f"Coupling[{r}][{c}]", c_json[r][c], v, f"{v:.{decimals}f}")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    if val_fails > 0:
        print(f"  Verification FAILED: {val_fails} error(s), {prec_warns} warning(s).")
        return False
    elif prec_warns > 0:
        print(f"  Verification PASSED with {prec_warns} warning(s).")
        return True
    else:
        print("  Verification PASSED!")
        return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 verify_dh.py <original-4xxxxx> <json_path>")
        sys.exit(1)

    original_path = sys.argv[1]
    json_path     = sys.argv[2]

    # Extract instrument ID from either filename
    m = re.search(r'(\d{6})', os.path.basename(original_path))
    if not m:
        m = re.search(r'(\d{6})', os.path.basename(json_path))
    if not m:
        print("Error: cannot find 6-digit instrument ID in filenames.")
        sys.exit(1)

    inst_id = m.group(1)
    success = verify_dh(inst_id, original_path, json_path)
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
