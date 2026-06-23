"""Spiral-descent optimizer for 2D fiber-coupling / beam alignment.

Maximizes a noisy, expensive-to-sample objective (photodiode intensity vs. a
pair of mirror knobs) by sampling along an Archimedean spiral whose centre is
continuously dragged toward higher intensity. Compared with off-the-shelf
optimizers it keeps consecutive samples *close together in knob space*, which
minimizes physical motor travel — the dominant cost on the hardware.

The algorithm and its rationale are documented in ``doc/spiral.md``. The two
core ingredients (labels match the note):

    A. space-filling spiral  -- ``step_rdxy``: grow the radius along an
       Archimedean spiral so the plane is covered with short, adjacent moves.
    B. drag the centre uphill -- ``step_x0y0``: pull ``(x0, y0)`` toward the
       intensity-weighted centroid of recent samples (a noise-robust,
       gradient-like update).

This module is pure (no hardware imports) and safe to import. Running it as a
script (``python spiral.py``) shows a matplotlib demo that maximizes a tilted
2D Gaussian — the only way to watch the algorithm run off the Pi.
"""

from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt
import logging
import tqdm


@dataclass
class SpiralPathConfig:
    """Tunable parameters for :class:`SpiralPath`.

    The production values live in ``step_optimize.spiral_params``; the defaults
    here drive the ``__main__`` demo.
    """

    I_meaningful: float = 100  # intensity scale that counts as real signal
    D: float = 2  # initial spiral arm spacing (``d``)
    SPIRAL_RESOLUTION: int = 15  # samples per spiral loop (sets the angular step)
    SPIRAL_SPAN: int = 10  # total run length, in loops
    SINGLE_SPIRAL_SPAN: float = 4  # max loops one spiral runs before giving up
    MAX_X0Y0_DISPLACEMENT: float = 10  # clamp on per-step centre displacement
    N_LOOPS_BEFORE_RESET_ORIGIN: float = 0.5  # loops before origin resets are allowed
    COEF_I_RESET_ORIGIN: float = 2  # sample/best ratio that triggers an origin reset
    COEF_I_DECAY: float = 0.99  # per-step decay of the breakthrough bar I_max
    alpha: float = 0.03  # base learning rate for dragging the centre


def gaussian2d(x: float, y: float, mu, cov) -> float:
    """Evaluate a 2D Gaussian at ``(x, y)`` plus a tiny noise term.

    Used only by the ``__main__`` demo as a stand-in for the real (noisy)
    coupling objective.
    """
    inv_cov = np.linalg.inv(cov)
    det_cov = float(np.linalg.det(cov))
    r = np.array([x, y]).T - mu
    z = np.exp(-0.5 * (r @ inv_cov @ r.T))
    coeff = 1 / (2 * np.pi * np.sqrt(det_cov))
    return coeff * z + 0.00000001 * np.random.randn()


def covariance_matrix(sigma_a, sigma_b, theta):
    """Build a 2D covariance matrix for a Gaussian with principal-axis
    standard deviations ``sigma_a``/``sigma_b`` rotated by ``theta`` radians.

    Σ = R · Λ · Rᵀ, with rotation ``R`` and diagonal variances ``Λ``.
    """
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    Lambda = np.array([[sigma_a**2, 0], [0, sigma_b**2]])
    Sigma = np.dot(R, np.dot(Lambda, R.T))
    return Sigma


class SpiralPath:
    """Spiral-descent search over a 2D parameter space.

    Tunables are held in ``self.config`` (a :class:`SpiralPathConfig`); all
    other attributes are per-run state (the spiral geometry and the recorded
    traces). Typical use is ``maximize(function, x0, bounds, options)``.

    Recorded traces (``pts_x``, ``pts_y``, ``pts_I``, ``pts_x0``, ``pts_y0``,
    ``pts_ellipticity``, ``pts_r``, ``pts_d``, ``pts_alpha``) are kept for
    plotting/diagnostics.
    """

    def __init__(self, config: SpiralPathConfig = None):
        self.config = config or SpiralPathConfig()
        self.bounds = None
        self.callback_function = None
        self.init_vars()

    def init_vars(self):
        """Reset all run state and recorded traces, deriving the spiral
        geometry from ``self.config``."""
        self.n_iter = 0
        self.pts_x = []
        self.pts_y = []
        self.pts_I = []
        self.pts_x0 = []
        self.pts_y0 = []
        self.pts_ellipticity = []
        self.pts_r = []
        self.pts_d = []
        self.pts_alpha = []
        #
        self.r = 0
        self.d = self.config.D
        self.delta_theta = 2 * np.pi / self.config.SPIRAL_RESOLUTION
        self.I_max = self.config.I_meaningful
        self.x0 = 0
        self.y0 = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.theta_axis = 0
        #
        self.num_before_reset_origin = 0

    def load_options(self, settings):
        """Override the config from a :class:`SpiralPathConfig` or a dict of
        field overrides."""
        if isinstance(settings, SpiralPathConfig):
            self.config = settings
        else:
            for key, value in settings.items():
                setattr(self.config, key, value)

    def mean(self, ptr, len_mean) -> float:
        """Mean of the last ``len_mean`` entries of ``ptr``.

        Returns 0 until ``ptr`` has at least ``len_mean`` entries, or if the
        mean comes out NaN.
        """
        if len(ptr) < len_mean:
            return 0
        else:
            list_taken = np.array(ptr[-len_mean:])
            mean_val = np.mean(list_taken)
            if not np.isnan(mean_val):
                return mean_val
            else:
                return 0

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-float(x)))

    def bounded(self, r):
        """Clamp point ``r = (x, y)`` into ``self.bounds`` if bounds are set."""
        if self.bounds is None:
            return r
        else:
            # bounds: [(min_x, max_x), (min_y, max_y)]
            x, y = r
            x = max(self.bounds[0][0], min(self.bounds[0][1], x))
            y = max(self.bounds[1][0], min(self.bounds[1][1], y))
            return (x, y)

    def step_rdxy(self):
        """Advance the spiral one step (ingredient A).

        Grows the radius along the Archimedean spiral, applies the
        breakthrough origin-reset rule, adapts the arm spacing ``d``, and emits
        the next sample point ``(x, y)``.
        """
        cfg = self.config
        # >>> Grow the radius: Archimedean spiral r = d * (theta - theta_axis)
        self.r = self.d * (self.theta - self.theta_axis)

        # >>> Reset the origin on a breakthrough.
        # Only after the spiral has run N_LOOPS_BEFORE_RESET_ORIGIN loops: if
        # the latest sample beats the running best by COEF_I_RESET_ORIGIN,
        # recentre the spiral on that point and restart the arm there.
        if (
            self.num_before_reset_origin
            >= cfg.N_LOOPS_BEFORE_RESET_ORIGIN * cfg.SPIRAL_RESOLUTION
        ):
            mean_I_short = self.mean(self.pts_I, 1)
            if mean_I_short > self.I_max * cfg.COEF_I_RESET_ORIGIN:
                # Re-anchor theta so the spiral continues smoothly from (x, y).
                self.theta = np.arctan2(self.y - self.y0, self.x - self.x0)
                self.theta_axis = self.theta - 1
                #
                self.x0 = self.x
                self.y0 = self.y
                self.num_before_reset_origin = 0
                self.I_max = mean_I_short
                logging.info(f"Reset origin at {self.x}, {self.y}, I_max= {self.I_max}")
        # Let the breakthrough bar decay so it is never permanent, but never
        # below the meaningful-intensity floor.
        self.I_max = max(cfg.COEF_I_DECAY * self.I_max, cfg.I_meaningful)
        self.num_before_reset_origin += 1
        self.pts_r.append(self.r)

        # >>> Adapt the arm spacing d.
        # Far from the peak (low recent mean intensity) d grows so the spiral
        # spreads out faster; near the peak it stays tight.
        mean_I_long = self.mean(self.pts_I, cfg.SPIRAL_RESOLUTION)
        uds = np.arctan(1 - mean_I_long / cfg.I_meaningful) / (np.pi / 2)
        self.d = self.d * (1 + 0.0002 * uds)
        self.pts_d.append(self.d)

        # >>> Emit the next sample point on the spiral.
        x = self.x0 + self.r * np.cos(self.theta)
        y = self.y0 + self.r * np.sin(self.theta)
        self.x, self.y = self.bounded((x, y))
        self.pts_x.append(self.x)
        self.pts_y.append(self.y)

        # >>> Advance the angle by one step (SPIRAL_RESOLUTION points per loop).
        self.delta_theta = 2 * np.pi / cfg.SPIRAL_RESOLUTION
        self.theta += self.delta_theta

    def step_x0y0(self):
        """Drag the spiral centre toward higher intensity (ingredient B).

        Over the last ``SPIRAL_RESOLUTION`` samples, compute the
        intensity-weighted mean displacement from ``(x0, y0)``, clamp it, and
        step the centre by ``alpha * mean_d``. ``alpha`` is scaled up when the
        recent samples are bright and anisotropic (``ellipticity``), so the
        centre moves decisively only when there is a real gradient to follow.
        """
        cfg = self.config
        if self.n_iter > cfg.SPIRAL_RESOLUTION:
            take_pts_x = np.array(self.pts_x[-cfg.SPIRAL_RESOLUTION :])
            take_pts_y = np.array(self.pts_y[-cfg.SPIRAL_RESOLUTION :])
            take_pts_I = np.array(self.pts_I[-cfg.SPIRAL_RESOLUTION :])
            #
            sum_I = np.sum(take_pts_I)
            mean_I = np.mean(take_pts_I)
            # Intensity-weighted mean displacement from the current centre.
            xI_sum = np.sum([(x - self.x0) * I for x, I in zip(take_pts_x, take_pts_I)])
            yI_sum = np.sum([(y - self.y0) * I for y, I in zip(take_pts_y, take_pts_I)])
            mean_dx = xI_sum / sum_I
            mean_dy = yI_sum / sum_I
            # Relative spread of recent intensities (a gradient/anisotropy cue).
            std_I = np.std(take_pts_I)
            ellipticity = std_I / mean_I
            #
            self.pts_ellipticity.append(ellipticity)
        else:
            mean_I = 0
            mean_dx = 0
            mean_dy = 0
            ellipticity = 0

        # Clamp the displacement so a single noisy batch can't jump the centre.
        mean_dx = max(
            -cfg.MAX_X0Y0_DISPLACEMENT, min(cfg.MAX_X0Y0_DISPLACEMENT, mean_dx)
        )
        mean_dy = max(
            -cfg.MAX_X0Y0_DISPLACEMENT, min(cfg.MAX_X0Y0_DISPLACEMENT, mean_dy)
        )
        # Only move the centre once recent signal is meaningful; scale the step
        # up with both anisotropy and brightness.
        if mean_I > cfg.I_meaningful:
            alpha = (
                cfg.alpha
                * (np.pi / 2 + np.arctan(ellipticity))
                * (np.pi / 2 + np.arctan(mean_I / cfg.I_meaningful))
            )
        else:
            alpha = 0
        self.pts_alpha.append(alpha)
        #
        x0 = self.x0 + alpha * mean_dx
        y0 = self.y0 + alpha * mean_dy
        self.x0, self.y0 = self.bounded((x0, y0))
        self.pts_x0.append(self.x0)
        self.pts_y0.append(self.y0)

    def step(self):
        """Run a single iteration: advance the spiral, sample, drag the centre.

        Returns ``False`` once the current spiral has spanned
        ``SINGLE_SPIRAL_SPAN`` loops since the last origin reset, otherwise
        ``True``.
        """
        cfg = self.config
        if (
            self.num_before_reset_origin
            < cfg.SINGLE_SPIRAL_SPAN * cfg.SPIRAL_RESOLUTION
        ):
            self.step_rdxy()
        else:
            return False
        #
        self.I = self.callback_function((self.x, self.y))
        self.pts_I.append(self.I)
        #
        self.step_x0y0()
        #
        self.n_iter += 1
        return True

    def maximize(self, function, x0, bounds, options=None):
        """Maximize ``function`` over the 2D box ``bounds`` by spiral descent.

        Args:
            function: objective ``f((x, y)) -> intensity`` to maximize.
            x0: starting point, also the initial spiral centre.
            bounds: ``[(min_x, max_x), (min_y, max_y)]`` (or None for no clamp).
            options: optional config override — a :class:`SpiralPathConfig` or
                a dict of field overrides applied via ``load_options``.

        Runs for ``SPIRAL_RESOLUTION * SPIRAL_SPAN`` iterations (or until
        ``step`` reports the spiral has finished) and returns the final
        centre ``(x0, y0)``.
        """
        if options is not None:
            self.load_options(options)
        # Re-derive run state from the (possibly updated) config.
        self.init_vars()
        self.callback_function = function
        self.x, self.y = x0
        self.x0, self.y0 = x0
        self.bounds = bounds
        self.I_max = self.callback_function((self.x, self.y))
        #
        total = self.config.SPIRAL_RESOLUTION * self.config.SPIRAL_SPAN
        with tqdm.tqdm(total=total) as pbar:
            while self.n_iter < total:
                if self.step():
                    pbar.update(1)
                else:
                    break
        #
        return (self.x0, self.y0)


if __name__ == "__main__":
    # Hardware-free demo: maximize a tilted 2D Gaussian and plot the spiral
    # sample trace (red) and the centre trajectory (blue).
    x = np.linspace(-40, 40, 100)
    y = np.linspace(-40, 40, 100)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros(X.shape)
    mu = np.array([8, 10])
    cov = covariance_matrix(10, 4, np.pi / 3)
    #
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Z[i, j] = gaussian2d(X[i, j], Y[i, j], mu, cov)

    plt.contourf(X, Y, Z)

    sp = SpiralPath()
    callback_function = lambda xy: gaussian2d(xy[0], xy[1], mu, cov)
    sp.maximize(callback_function, x0=(0, 0), bounds=[(-40, 40), (-40, 40)])

    plt.plot(sp.pts_x, sp.pts_y, "r")
    plt.plot(sp.pts_x0, sp.pts_y0, "b")
    plt.axis("equal")
    plt.figure()
    plt.plot(sp.pts_ellipticity)
    plt.show()
