# The Jacobian — Keeping Two Beams Mode-Matched

This note explains **what** the Jacobian means for our mirror mounts, **why** we
calibrate it numerically instead of from geometry, and **how** the calibration
in `src/calibrate_jacobian.py` works.

## The setup: two paths, eight knobs

There are **two beam paths** that must stay **mode-matched** (e.g. two
counter-propagating beams overlapping through the cavity / fiber). Each path is
steered by **two mirrors × two knobs = four knobs**:

- Path **A** (upper): knobs `A1…A4` = `x, y, xdot, ydot`
- Path **B** (lower): knobs `B1…B4` = `x, y, xdot, ydot`

where on each mirror one knob is mainly a **position** adjust (`x`, `y`) and the
other mainly an **angle** adjust (`xdot`, `ydot`).

![Beam-path diagram: A1–A4 and B1–B4 steer two crossing beams](figs/jacobian_beam_path_diagram.jpg)

![The physical mounts on the optical table](figs/optical_table_setup.jpg)

## What "the Jacobian" is here

Suppose we **move the four knobs of one path** (call them the **master** knobs,
$A$) — for example because we are walking the beam to a new target. To keep the
two beams mode-matched, the **other path's knobs** (the **slave** knobs, $B$)
must move too. The relationship, to first order, is a **4×4 Jacobian matrix**:

$$
J_{ji} \;=\; \frac{\partial B_j}{\partial A_i}
\qquad\Longrightarrow\qquad
\Delta B \;=\; J \,\Delta A .
$$

Once $J$ is known, you can drive the master knobs freely and have the slave
knobs **follow automatically** to preserve coupling — no re-optimization needed.

In code this is exactly what [`compose_para`](../src/servo_util.py) does: given a
master move `dA` it sets the slave knobs by `dB = J·(dA − offset)`, where
`jac_master_mask` selects which channels are the masters. The
[spiral/L-BFGS optimizer](spiral.md) is what finds the optimal $B$ for each
imposed $A$ during calibration.

## Why a numeric ("no-model") Jacobian?

The knob geometry *can* be written analytically (a model-based Jacobian, as
formulated for the ideal mount). We deliberately use the **numeric, no-model**
version instead because:

1. **The knobs are coupled.** The mirror's x- and y-tilt are **not** controlled
   independently by the x and y knobs — there is cross-coupling the simple model
   ignores.
2. **The model only holds near perfect coupling.** You can compute knob
   positions analytically *on* the coupling condition, but once the system is
   **off-coupling** (which is exactly when you need help) the analytic
   expression no longer applies.

A numeric Jacobian, measured on the real hardware, captures the actual coupling
including the 3D-printed-frame imperfections.

## How calibration works (`calibrate_jacobian.py`)

The core loop turns the physics statement *"for any master setting $A$, find the
slave setting $B$ that maximizes coupling"* into data:

1. **Impose a master offset.** Choose an offset vector for the master path's
   knobs. The offset *type* is selectable:
   - `pm`  — push one knob by ±`normd` (coordinate directions),
   - `rand` — a random unit direction scaled to `normd`,
   - `lin` — a random combination of known coupling directions (`vecs`),
   - `zero` — no offset (re-optimize in place).
2. **Optimize the slave path.** With the master held at the imposed offset, run
   the staged [spiral + L-BFGS-B optimization](spiral.md) over the slave knobs:

   ```python
   zero = step_optimize(..., pos_mask=X_Y_MASK,    bounds_single=(-100,100))
   zero = step_optimize(..., pos_mask=X_XDOT_MASK)                 # spiral
   zero = step_optimize(..., pos_mask=Y_YDOT_MASK)                 # spiral
   zero = step_optimize(..., pos_mask=POS_ALL_MASK, method='L-BFGS-B')
   ```

   This yields the optimal slave knob positions for that master offset.
3. **Record the pair.** Save `(offset, optimal_slave_positions, intensity)` into
   the dataset (`jacobian_<type>_<normd>.npy`).
4. **Repeat** over many offsets / directions to build up a cloud of
   $(\Delta A, \Delta B)$ pairs, then **least-squares fit** $J$ from
   $\Delta B = J\,\Delta A$ (the fit itself is done in the analysis notebooks).

> In the shipped script, `MASTER = "B"`: path **B** receives the imposed offset
> (`offset_mask = B_POS_ALL_MASK`) and path **A** is the one optimized. Which
> path is master is just a configuration choice — the roles are symmetric.

A single calibrated 4×4 Jacobian (here from the small `pm_10` offset set) looks
like:

![A calibrated 4×4 Jacobian (∂B/∂A) from the pm_10 offset set](figs/jacobian_calibrated_pm10.png)

## Running the optimization in code

This section maps the loop above onto the actual code in
`src/calibrate_jacobian.py` and the helpers it calls. To run it:

```bash
python calibrate_jacobian.py      # from src/ (touches hardware immediately)
```

### The objective: `callback_func`

Every optimizer call goes through one objective that turns a reduced parameter
vector into a servo move and a photodiode reading:

```python
def callback_func(para, pos_mask, zero=None, jac=None, jac_master_mask=None, **kw):
    para_nr_move = compose_para(para, pos_mask, zero, jac, jac_master_mask, **kw)
    servos.set_angle(list(para_nr_move))          # move the motors
    z = np.mean([MCP3424_fiber.convert_and_read() # read photodiode twice, average
                 for _ in range(2)])
    return tuple(para), z                          # (param, intensity)
```

- `pos_mask` selects which of the 8 channels `para` addresses (the masks in
  `servo_const.py`, e.g. `A_X_XDOT_MASK`).
- `zero` is the current origin the parameter steps from.
- `jac` / `jac_master_mask` (optional) make the **slave** knobs follow the
  **master** knobs while scanning — see `compose_para` below.

> ⚠️ `callback_func` is **copy-pasted** into `clip_scan.py`,
> `calibrate_jacobian.py`, and `callback_func.py`; it relies on module-level
> `servos` and `MCP3424_fiber`, so the standalone `callback_func.py` copy only
> works inside a namespace that defines them.

### `compose_para` — where the Jacobian is applied

`compose_para` (`servo_util.py`) builds the full 8-channel angle command:

1. Start from `zero`, add the reduced step `para` on the masked channels
   (`nraddr`).
2. If a Jacobian is supplied, set the slave channels by
   `dB = J·(dA − offset)` (`jac_master_mask` picks the masters,
   `1 − jac_master_mask` the slaves), optionally offset by `jac_x0`.

So the *same* function (a) imposes the master offset during setup and (b) holds
the slaves on the Jacobian during a scan — it is the one place the matrix is
used, both to **calibrate** and later to **apply** `J`.

### One calibration sample

For each sample `i` the script does:

```python
MASTER = "B"                                   # B is held at the offset; A is optimized
offset_mask = B_POS_ALL_MASK                   # the master channels
offset = cord_pm_offset(...)                   # or random_norm_offset / lin_comb_offset / zeros

# impose the master offset (and apply an assumed J to the slaves, if any)
zero = compose_para(para=offset, pos_mask=offset_mask, zero=zero,
                    jac=jac_assume, jac_master_mask=offset_mask, jac_x0=jac_x0)

# optimize the slave path on top of that origin (staged: see spiral.md)
zero = step_optimize(servos, callback_func, pos_mask=A_X_Y_MASK,  zero=zero, bounds_single=(-100,100))
zero = step_optimize(servos, callback_func, pos_mask=A_X_XDOT_MASK, zero=zero)               # spiral
zero = step_optimize(servos, callback_func, pos_mask=A_Y_YDOT_MASK, zero=zero)               # spiral
zero = step_optimize(servos, callback_func, pos_mask=A_POS_ALL_MASK, zero=zero, method='L-BFGS-B')

_, I = callback_func(zero, pos_mask=POS_ALL_MASK)        # final intensity
dataset[tuple(offset)].append((list(zero), I))          # record (offset → optimal slaves)
np.save(filename, dataset)
```

`step_optimize` runs one stage via `pts_iterator` (spiral / L-BFGS-B / Powell)
and **only commits the new origin if the final intensity stays ≥ 70 % of the best
seen** — see [spiral.md](spiral.md).

### Knobs to set before running

Near the top of the `for i in range(N)` loop:

| Variable | Meaning |
|----------|---------|
| `MASTER` | `"A"` or `"B"` — which path is held at the offset (the other is optimized). |
| `offset_type` | `'pm'` (coordinate ±), `'rand'` (random unit), `'lin'` (combination of known coupling vectors), `'zero'` (re-optimize in place). |
| `normd` | Offset magnitude (the master step size; grow it for [extrapolation](#extrapolation-bootstrapping-from-small-to-large-steps)). |
| `N` | Number of samples to collect. |
| `jac_assume` / `jac_x0` | A previously-fit `J` to predict slave starts (currently set to `None`). |
| `filename` | `jacobian_<type>_<normd>.npy` — the accumulated dataset; the **matrix fit** `ΔB = J·ΔA` is done afterward in the analysis notebooks. |

## Extrapolation: bootstrapping from small to large steps

A Jacobian fit from **small** master deviations is noisy (the optimizer's
scatter is comparable to the signal). To get an accurate $J$ over a **large**
working range, bootstrap:

1. Run the optimization with **small** offsets; extract a rough $J$.
2. **Increase the step size.** For each larger offset, use the *previous* $J$ to
   **predict the starting slave positions** (`dB = J·dA`), then run the
   optimization from that predicted start. Starting near the optimum keeps the
   slave search short and reliable even at large offsets.
3. Iterate. As the step size grows, $J$ becomes progressively more accurate.

![Jacobians fit at increasing step size (pm_30 → pm_250) converge](figs/jacobian_vs_step_size.png)

The matrices stabilize as the step size grows; their pairwise similarity
(correlation $-\lVert A-B\rVert^2$) confirms convergence to a consistent $J$:

![Pairwise correlation of the step-size Jacobians](figs/jacobian_correlation_matrix.png)

## Using the calibrated Jacobian

Pass the fitted matrix back into the alignment code via `compose_para`
(`jac=J`, `jac_master_mask=<master channels>`): moving the master knobs then
auto-sets the slave knobs by `dB = J·(dA − offset)`, holding the two beams
mode-matched across the calibrated region. This is the building block for
maintaining mode matching while, e.g., walking the lattice beam onto the cavity
axis or interpolating the MOT trapping position toward the aligned beam.

## Validation: does the calibrated Jacobian hold?

A fitted `J` is only useful if moving the **master** keeps the **slave**
aligned. To test it, impose a master move *along the calibrated coupling
direction* and check the slave's optimum does not drift. The script at the end of
the notes scans the `B_X_XDOT` clip while stepping a Y/Ydot offset that follows
the calibrated coupling and fits the clip center `mu` at each step:

```python
for i in range(-2, 4):
    zero = np.array([0, 0, 0, 0, 0, 100*i, 0, -67*i])   # step along the Y↔Ydot coupling (≈ -0.67)
    X, Y, Z = motor_2d_scan(N_pts, SCAN_RANGE, servos, cf, accept_func=accept_func_BXXDOT)
    mu = popt_get_mu_cov(fit_and_plot_smooth_heaviside(X, Y, Z))   # extracted clip center
```

**More calibration data ⇒ a flatter region.** A `J` fit from `pm10` data alone
vs from `pm10 + lin_70` — the latter keeps the transmitting band straighter
across the offset range:

| Calibrated `pm10 + lin_70` | Calibrated `pm10` only |
|---|---|
| ![clip band stays straight across offsets](figs/jacobian_validation_band_pm10_lin70.png) | ![clip band bows more with offset](figs/jacobian_validation_band_pm10.png) |

**The clip stays centered over the working range, then breaks.** Overlaying the
row-sum clip profiles across the six offsets, the flat-topped plateaus stay
centered through the middle of the range and only collapse into a skewed
([fat-tail](application.md)) bump at the extreme offsets — the edge of the
calibrated region:

![Overlaid clip profiles across offsets: plateaus centered in range, skewed at the extremes](figs/jacobian_validation_clip_profiles.png)

**Quantitatively**, the extracted center `mu` vs offset index is ≈ flat near zero
through the middle (the slave stays aligned → `J` is good) with a sharp drop at
the ends (`J` breaks down outside the calibrated range):

![Extracted clip center mu vs imposed offset index — flat in range, drops at the edges](figs/jacobian_validation_mu_vs_offset.png)

This is the quantitative form of *"with the calibrated Jacobian we maintain good
mode matching within this region"* — and a direct argument for the
[extrapolation](#extrapolation-bootstrapping-from-small-to-large-steps) loop,
which widens the region where `mu` stays flat.

See also [spiral.md](spiral.md) for the optimizer used at each calibration point,
and [motor.md](motor.md) for the encoder/de-hysteresis behavior that limits
calibration accuracy.
