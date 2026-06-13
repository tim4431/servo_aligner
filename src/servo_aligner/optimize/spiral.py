"""Spiral-descent optimizer for noisy 2D objectives.

``SpiralPath`` walks an outward space-filling spiral whose center is
continuously dragged toward higher intensity; when a sample beats the
running maximum by ``COEF_I_RESET_ORIGIN`` the spiral restarts around it.
This is the custom algorithm developed for beam alignment — the tuning
constants and update rules are lab-validated (see ``doc/spiral.md``).
Do not change the numerics.
"""

from __future__ import annotations

import logging

import numpy as np
import tqdm

logger = logging.getLogger(__name__)


class SpiralPath:
    """2D spiral-descent maximizer (see module docstring)."""

    def __init__(self):
        self._I_meaningful = 100
        self.bounds = None
        self._SPIRAL_RESOLUTION = 15  # points per circle
        self.SPIRAL_SPAN = 10
        self.SINGLE_SPIRAL_SPAN = 4
        self.MAX_X0Y0_DISPLACEMENT = 10
        self.N_LOOPS_BEFORE_RESET_ORIGIN = 0.5
        self.COEF_I_RESET_ORIGIN = 2
        self.COEF_I_DECAY = 0.99
        self.alpha = 0.03
        self._D = 2
        self.I_max = self._I_meaningful
        #
        self.init_vars()
        #
        self.callback_function = None

    @property
    def SPIRAL_RESOLUTION(self):
        return self._SPIRAL_RESOLUTION

    @SPIRAL_RESOLUTION.setter
    def SPIRAL_RESOLUTION(self, SPIRAL_RESOLUTION):
        self._SPIRAL_RESOLUTION = SPIRAL_RESOLUTION
        self.delta_theta = 2 * np.pi / self._SPIRAL_RESOLUTION

    @property
    def I_meaningful(self):
        return self._I_meaningful

    @I_meaningful.setter
    def I_meaningful(self, I_meaningful):
        self._I_meaningful = I_meaningful
        self.I_max = self._I_meaningful

    @property
    def D(self):
        return self._D

    @D.setter
    def D(self, D):
        self._D = D
        self.d = self._D

    def init_vars(self):
        self.n_iter = 0
        self.pts_x = []
        self.pts_y = []
        self.pts_I = []
        self.pts_x0 = []
        self.pts_y0 = []
        self.pts_ellipcity = []
        self.pts_r = []
        self.pts_d = []
        self.pts_alpha = []
        #
        self.r = 0
        self.d = self._D
        self.x0 = 0
        self.y0 = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.theta_axis = 0
        #
        self.num_before_reset_origin = 0

    def load_options(self, settings: dict):
        for key, value in settings.items():
            setattr(self, key, value)

    def mean(self, ptr, len_mean) -> float:
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
        if self.bounds is None:
            return r
        else:
            # bounds: [(min_x, max_x), (min_y, max_y)]
            x, y = r
            x = max(self.bounds[0][0], min(self.bounds[0][1], x))
            y = max(self.bounds[1][0], min(self.bounds[1][1], y))
            return (x, y)

    def step_rdxy(self):
        """Update the r, d, x, y values."""
        # >>> update r value
        self.r = self.d * (self.theta - self.theta_axis)
        if (
            self.num_before_reset_origin
            < self.N_LOOPS_BEFORE_RESET_ORIGIN * self._SPIRAL_RESOLUTION
        ):
            pass
        else:
            mean_I_short = self.mean(self.pts_I, 1)
            if mean_I_short > self.I_max * self.COEF_I_RESET_ORIGIN:
                self.theta = np.arctan2(self.y - self.y0, self.x - self.x0)
                self.theta_axis = self.theta - 1
                #
                self.x0 = self.x
                self.y0 = self.y
                self.num_before_reset_origin = 0
                self.I_max = mean_I_short
                logger.info(f"Reset origin at {self.x}, {self.y}, I_max= {self.I_max}")
        self.I_max = max(self.COEF_I_DECAY * self.I_max, self._I_meaningful)
        self.num_before_reset_origin += 1
        self.pts_r.append(self.r)

        # >>> update d value
        mean_I_long = self.mean(self.pts_I, self._SPIRAL_RESOLUTION)
        uds = np.arctan(1 - mean_I_long / self._I_meaningful) / (np.pi / 2)
        self.d = self.d * (1 + 0.0002 * uds)
        self.pts_d.append(self.d)

        # >>> update x, y value
        x = self.x0 + self.r * np.cos(self.theta)
        y = self.y0 + self.r * np.sin(self.theta)
        self.x, self.y = self.bounded((x, y))
        self.pts_x.append(self.x)
        self.pts_y.append(self.y)

        # >>> update theta value
        self.delta_theta = 2 * np.pi / self._SPIRAL_RESOLUTION
        self.theta += self.delta_theta

    def step_x0y0(self):
        """Drag the spiral center toward the intensity-weighted displacement."""
        if self.n_iter > self._SPIRAL_RESOLUTION:
            take_pts_x = np.array(self.pts_x[-self._SPIRAL_RESOLUTION :])
            take_pts_y = np.array(self.pts_y[-self._SPIRAL_RESOLUTION :])
            take_pts_I = np.array(self.pts_I[-self._SPIRAL_RESOLUTION :])
            #
            sum_I = np.sum(take_pts_I)
            mean_I = np.mean(take_pts_I)
            xI_sum = np.sum([(x - self.x0) * I for x, I in zip(take_pts_x, take_pts_I)])
            yI_sum = np.sum([(y - self.y0) * I for y, I in zip(take_pts_y, take_pts_I)])
            mean_dx = xI_sum / sum_I
            mean_dy = yI_sum / sum_I
            std_I = np.std(take_pts_I)
            ellipcity = std_I / mean_I
            #
            self.pts_ellipcity.append(ellipcity)
        else:
            mean_I = 0
            mean_dx = 0
            mean_dy = 0
            ellipcity = 0
        #
        mean_dx = max(
            -self.MAX_X0Y0_DISPLACEMENT, min(self.MAX_X0Y0_DISPLACEMENT, mean_dx)
        )
        mean_dy = max(
            -self.MAX_X0Y0_DISPLACEMENT, min(self.MAX_X0Y0_DISPLACEMENT, mean_dy)
        )
        if mean_I > self.I_meaningful:
            alpha = (
                self.alpha
                * (np.pi / 2 + np.arctan(ellipcity))
                * (np.pi / 2 + np.arctan(mean_I / self._I_meaningful))
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

    def step(self) -> bool:
        if self.num_before_reset_origin < self.SINGLE_SPIRAL_SPAN * self._SPIRAL_RESOLUTION:
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
        """Run the spiral search and return the final center ``(x0, y0)``.

        Args:
            function: Objective ``(x, y) -> intensity`` (scalar, maximized).
            x0: Starting point ``(x, y)``.
            bounds: ``[(min_x, max_x), (min_y, max_y)]`` or ``None``.
            options: Attribute overrides (e.g. the ``spiral`` section of the
                optimizer config), applied via :meth:`load_options`.
        """
        self.callback_function = function
        self.x, self.y = x0
        self.x0, self.y0 = x0
        self.bounds = bounds
        self.load_options(options or {})
        self.I_max = self.callback_function((self.x, self.y))
        #
        with tqdm.tqdm(total=self.SPIRAL_RESOLUTION * self.SPIRAL_SPAN) as pbar:
            while self.n_iter < self.SPIRAL_RESOLUTION * self.SPIRAL_SPAN:
                if self.step():
                    pbar.update(1)
                else:
                    break
        #
        return (self.x0, self.y0)
