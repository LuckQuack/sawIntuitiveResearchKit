# dVRK Instrument DH Parameters Converter & Verifier

Tools for converting MATLAB-like Denavit-Hartenberg (DH) instrument parameters into the standard dVRK tool JSON configuration format, as well as verifying existing or generated JSON files against original parameter definitions.

## Scripts

### 1. convert_dh.py
Converts one or more MATLAB-style parameter files (`original-4xxxxx`) into dVRK tool JSON files.
- **Usage**: 
  ```bash
  python3 convert_dh.py <input_dir_or_file> <output_dir>
  ```
- **Example**:
  ```bash
  python3 convert_dh.py /Users/anton/devel/dh /Users/anton/devel/dh/generated_json
  ```

### 2. verify_dh.py
Validates a dVRK JSON file against its corresponding `original-4xxxxx` parameter definition.
- **Usage**:
  ```bash
  python3 verify_dh.py <original-4xxxxx> <json_path>
  ```
- **Validation Rules**:
  - **Fails (`[FAIL]`)**: Mismatches in DH joint parameters (`alpha`, `A`, `theta`, `D`, `offset`) or the coupling matrix elements beyond `1e-4` tolerance.
  - **Warnings (`[WARN]`)**: Value mismatches on joint/jaw range limits (`qmin`/`qmax`/`ftmax`), or string precision/formatting differences (e.g. `-1.5708` vs `-1.570796...`).

---

## Mapping Details

Below is the detailed mapping from the MATLAB-style parameters in `original-4xxxxx` files to the output JSON keys.

### 1. DH Joint Parameters (`DH.joints`)
The `data.dh_params` rows correspond to the joints:
- **Joint 1 (roll)**: `data.dh_params(INDEX1,:)`
- **Joint 2 (wrist_pitch)**: `data.dh_params(INDEX2,:)`
- **Joint 3 (wrist_yaw)**: `data.dh_params(INDEX3,:)`

Each row `[type, A, alpha, ?, D, theta, ?]` is mapped as follows:
- **`alpha`**: Col 3 value $\times \frac{\pi}{2}$ radians (e.g., `-1` $\rightarrow$ `-1.5708`).
- **`A`**: Col 2 value in meters.
- **`theta`**: `0.0` radians.
- **`D`**: Col 5 value in meters.
  - *Classic Counterparts (`400xxx`)*: If `dh_params` is absent, Joint 1 `D` defaults to `0.4162` (or `0.4024` for Needle Driver `400117`).
  - *S/Si Counterparts (`420xxx`)*: If absent, Joint 1 `D` defaults to `0.4670` (or `0.4532` for `420117`).
- **`offset`**: Col 6 value $\times \frac{\pi}{2}$ radians (e.g., `-1` $\rightarrow$ `-1.5708`).

### 2. Coupling Matrix (`coupling.ActuatorToJointPosition`)
- For **Spatulas and Hooks** (where close torque is `0.0`), a dVRK-specific hack coupling matrix is applied:
  $$C = \begin{bmatrix} c_{11} & 0 & 0 & 0 \\ 0 & c_{22} & 0 & 0 \\ 0 & c_{32} & -c_{43} & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
  where $c_{11}$, $c_{22}$, $c_{32}$ are taken from `data.coupling(1..3,:)` and $c_{43}$ is negated from column 3 of `data.coupling(INDEX4,:)`.
- For other instruments, the matrix matches the `data.coupling(1..4,:)` coefficients directly.

### 3. Joint & Jaw Limits (`qmin`, `qmax`, `ftmax`)
- **Joint Limits**: Taken directly from `data.joint.signal_range(LOWER_LIMIT,:)` and `(UPPER_LIMIT,:)`.
- **Jaw Limits**:
  - **Spatulas/Hooks**:
    - `"qmin"`: `0.0`, `"qmax"`: `0.5`, `"ftmax"`: `0.0`
  - **Other Instruments**:
    - `"qmin"`: `-0.698132` ($-40^\circ$) for Fenestrated Bipolar, else `-0.349066` ($-20^\circ$).
    - `"qmax"`: Upper limit for joint 4 (from `signal_range`) for clip appliers, else `1.39626` ($80^\circ$).
    - `"ftmax"`: Max close torque from `signal_range` for clip appliers, else `0.19`.

### 4. Engage Positions (`tool_engage_position`)
Angles used to engage the tool (in radians):
- **Lower/Upper Roll Bounds**: Consistently set to `[-4.5378, 4.5378]`.
- **Pitch & Yaw Bounds**: 
  - $15^\circ$ (`0.261799`) for Spatulas/Hooks.
  - $5^\circ$ (`0.087266`) for Bowel Graspers (`400127`), Thoracic Graspers (`400208`, `420208`), and Clip Appliers (`400230`, `420230`, `420327`).
  - $10^\circ$ (`0.174533`) for all other forceps/needle driver instruments.
