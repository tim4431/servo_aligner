"""Capture golden values from the legacy flat modules in src/.

Run ONCE from the repo root, before the legacy modules are deleted:

    MPLBACKEND=Agg .venv/bin/python tests/golden/_capture_from_legacy.py

The output (golden_values.json) is committed and used by the test suite to
assert that the refactored package reproduces the legacy numerics exactly.
This script is kept for provenance; it cannot run once src/*.py are gone.
"""
import json
import sys
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC))

import servo_util  # noqa: E402
import servo_const  # noqa: E402
import spiral  # noqa: E402
import fit_gaussian  # noqa: E402
import numeric_sim  # noqa: E402
from pts_iterator import pts_iterator  # noqa: E402
import step_optimize  # noqa: E402

golden = {}

# ---------------------------------------------------------------- vectors
golden["a2p"] = {str(a): servo_util.a2p(a) for a in [0, 90, -90, 17.3, 360, -360.5]}
golden["r2nd"] = servo_util.r2nd([-5, 4], [1, 0, 1, 0]).tolist()
golden["r2nr"] = servo_util.r2nr([-5.5, 4.25], [1, 0, 1, 0]).tolist()
golden["nrselr"] = servo_util.nrselr([10.0, 20.0, 30.0, 40.0], [0, 1, 0, 1]).tolist()
golden["nrmodr"] = servo_util.nrmodr(
    [1.0, 2.0, 3.0, 4.0], [9.5, -9.5], [1, 0, 0, 1]
).tolist()
golden["nraddr"] = servo_util.nraddr(
    [1.0, 2.0, 3.0, 4.0], [0.5, -0.5], [0, 1, 1, 0]
).tolist()
golden["ndmodr"] = servo_util.ndmodr(
    [2048, 2048, 2048, 2048], [45.0, -45.0], [1, 0, 0, 1]
).tolist()
golden["format_para"] = servo_util.format_para([1.234, -5.678])

X = np.arange(12, dtype=float).reshape(3, 4)
X_zig, index_map = servo_util.create_zigzag_X(X)
golden["zigzag"] = {"X_zig": X_zig.tolist(), "index_map": index_map.tolist()}

# ----------------------------------------------------------- compose_para
masks = servo_const
golden["compose_plain"] = servo_util.compose_para(
    [3.0, -7.0],
    masks.A_X_XDOT_MASK,
    zero=np.array([1, 2, 3, 4, 5, 6, 7, 8], dtype=float),
).tolist()

jac = np.array(
    [
        [0.5, -0.1, 0.0, 0.2],
        [0.0, 0.3, -0.2, 0.1],
        [0.1, 0.0, 0.4, -0.3],
        [-0.2, 0.1, 0.0, 0.6],
    ]
)
golden["compose_jac"] = servo_util.compose_para(
    [10.0, -20.0, 5.0, 2.5],
    masks.B_POS_ALL_MASK,
    zero=np.array([1, 2, 3, 4, 5, 6, 7, 8], dtype=float),
    jac=jac,
    jac_master_mask=masks.B_POS_ALL_MASK,
    jac_x0=np.array([0.1, 0.2, 0.3, 0.4]),
).tolist()

golden["compose_jac_offset"] = servo_util.compose_para(
    [10.0, -20.0, 5.0, 2.5],
    masks.B_POS_ALL_MASK,
    zero=np.zeros(8),
    jac=jac,
    jac_master_mask=masks.B_POS_ALL_MASK,
    jac_master_offset=np.array([1.0, 1.0, -1.0, -1.0]),
    jac_x0=np.array([0.1, 0.2, 0.3, 0.4]),
).tolist()

# all legacy mask constants, for channel-layout parity tests
golden["legacy_masks"] = {
    name: getattr(masks, name)
    for name in dir(masks)
    if name.endswith("_MASK")
}

# ----------------------------------------------------------------- spiral
np.random.seed(42)
mu = np.array([8.0, 15.0])
cov = spiral.covariance_matrix(4, 0.2, np.pi / 3)
sp = spiral.SpiralPath()
objective = lambda xy: spiral.gaussian2d(xy[0], xy[1], mu, cov)  # noqa: E731
result = sp.maximize(
    objective,
    x0=(0.0, 0.0),
    bounds=[(-20, 20), (-20, 20)],
    options=dict(step_optimize.spiral_params),
)
golden["spiral"] = {
    "result": list(result),
    "n_pts": len(sp.pts_x),
    "pts_x": list(sp.pts_x),
    "pts_y": list(sp.pts_y),
    "pts_I": list(sp.pts_I),
    "pts_x0": list(sp.pts_x0),
    "pts_y0": list(sp.pts_y0),
}

# ----------------------------------------------------------- pts_iterator
np.random.seed(7)
trace = []


def cb(para):
    z = float(np.exp(-((para[0] - 3.0) ** 2 + (para[1] + 2.0) ** 2) / 50.0))
    trace.append((tuple(para), z))
    return tuple(para), z


best_para, best_I = pts_iterator(
    N_var=2,
    callback_func=cb,
    p0=[0.0, 0.0],
    bounds=[(-50, 50), (-50, 50)],
    method="spiral",
    options=dict(step_optimize.spiral_params),
)
golden["pts_iterator_spiral"] = {
    "best_para": list(best_para),
    "best_I": best_I,
    "n_evals": len(trace),
}

np.random.seed(7)
trace2 = []


def cb2(para):
    z = float(np.exp(-((para[0] - 3.0) ** 2 + (para[1] + 2.0) ** 2) / 50.0))
    trace2.append((tuple(para), z))
    return tuple(para), z


best_para2, best_I2 = pts_iterator(
    N_var=2,
    callback_func=cb2,
    p0=[0.0, 0.0],
    bounds=[(-50, 50), (-50, 50)],
    method="L-BFGS-B",
    options=dict(step_optimize.BFGS_params),
)
golden["pts_iterator_lbfgsb"] = {
    "best_para": list(best_para2),
    "best_I": best_I2,
    "n_evals": len(trace2),
}

# ----------------------------------------------------------- step_optimize
np.random.seed(11)


def legacy_callback(para, pos_mask, zero=None, **kwargs):
    nr = servo_util.compose_para(para, pos_mask, zero)
    sel = servo_util.nrselr(nr, pos_mask)
    z = float(np.exp(-((sel[0] - 5.0) ** 2 + (sel[1] - 4.0) ** 2) / 200.0))
    return tuple(para), z


zero0 = np.zeros(8)
zero_new = step_optimize.step_optimize(
    None,
    legacy_callback,
    pos_mask=masks.A_X_XDOT_MASK,
    zero=zero0,
    method="spiral",
)
golden["step_optimize_accept"] = {"zero_new": list(np.asarray(zero_new, dtype=float))}

# ------------------------------------------------------------ fit_gaussian
x = np.linspace(-5, 5, 25)
y = np.linspace(-5, 5, 25)
GX, GY = np.meshgrid(x, y)
mu_t = np.array([0.7, -1.1])
cov_t = np.array([[2.0, 0.3], [0.3, 1.2]])
GZ = np.array(
    [
        [
            fit_gaussian.gaussian_2d_smooth_heaviside(xi, yi, mu_t, cov_t, 0.5, 0.2)
            for xi, yi in zip(xrow, yrow)
        ]
        for xrow, yrow in zip(GX, GY)
    ]
)
popt = fit_gaussian.fit_gaussian_2d_smooth_heaviside(GX, GY, GZ)
golden["fit_heaviside"] = {"popt": popt.tolist()}

popt_g = fit_gaussian.fit_gaussian_2d(
    GX,
    GY,
    np.array(
        [
            [fit_gaussian.gaussian_2d(xi, yi, mu_t, cov_t) for xi, yi in zip(xr, yr)]
            for xr, yr in zip(GX, GY)
        ]
    ),
)
golden["fit_gaussian"] = {"popt": popt_g.tolist()}

# ------------------------------------------------------------- numeric_sim
Z, xZ, tZ = numeric_sim.calc_data(
    crosstalk_matrix=numeric_sim.crosstalk_matrix,
    zero=np.zeros(4),
    scan_type="xxdot",
)
golden["numeric_sim"] = {
    "Z_sum": float(Z.sum()),
    "xZ_sum": float(xZ.sum()),
    "tZ_sum": float(tZ.sum()),
    "Z_samples": [
        [int(i), int(j), float(Z[i, j])]
        for i, j in [(0, 0), (40, 40), (39, 41), (41, 39), (20, 60), (79, 79)]
    ],
}
Zy, _, _ = numeric_sim.calc_data(
    crosstalk_matrix=numeric_sim.crosstalk_matrix,
    zero=np.array([10.0, -5.0, 3.0, 2.0]) * np.pi / 180,
    scan_type="yydot",
)
golden["numeric_sim_yydot"] = {"Z_sum": float(Zy.sum())}

out = Path(__file__).parent / "golden_values.json"
out.write_text(json.dumps(golden, indent=1))
print(f"wrote {out}")
print("spiral result:", golden["spiral"]["result"])
print("step_optimize zero_new:", golden["step_optimize_accept"]["zero_new"])
