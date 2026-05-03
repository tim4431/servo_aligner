import numpy as np
import matplotlib.pyplot as plt
import logging
import tqdm

def gaussian2d(x: float, y: float, mu, cov) -> float:
    inv_cov = np.linalg.inv(cov)
    det_cov = float(np.linalg.det(cov))
    r = np.array([x, y]).T - mu
    z = np.exp(-0.5 * (r @ inv_cov @ r.T))
    coeff = 1 / (2 * np.pi * np.sqrt(det_cov))
    return coeff * z + 0.00000001 * np.random.randn()


def covariance_matrix(sigma_a, sigma_b, theta):
    # Rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    # Diagonal matrix of variances
    Lambda = np.array([[sigma_a**2, 0], [0, sigma_b**2]])

    # Covariance matrix: Σ = R * Λ * R^T
    Sigma = np.dot(R, np.dot(Lambda, R.T))

    return Sigma


class SpiralPath:

    def __init__(self):
        self._I_meaningful = 100
        self.bounds=None
        self._SPIRAL_RESOLUTION = 15  # 100 pts per circle
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
        self.theta=0
        self.theta_axis=0
        #
        self.num_before_reset_origin = 0

    def load_options(self, settings):
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
            # bounds: [(min_x,max_x),(min_y,max_y)]
            x, y = r
            x = max(self.bounds[0][0], min(self.bounds[0][1], x))
            y = max(self.bounds[1][0], min(self.bounds[1][1], y))
            return (x, y)

    def step_rdxy(self):
        """
        Update r, d, x, y value
        """
        # >>> update r value
        # self.r += self.d * self.delta_theta
        self.r  = self.d * (self.theta-self.theta_axis)
        if (
            self.num_before_reset_origin
            < self.N_LOOPS_BEFORE_RESET_ORIGIN * self._SPIRAL_RESOLUTION
        ):
            pass
        else:
            mean_I_short = self.mean(self.pts_I, 1)
            if mean_I_short > self.I_max * self.COEF_I_RESET_ORIGIN:
                # self.r = self._D
                self.theta=np.arctan2(self.y-self.y0,self.x-self.x0)
                self.theta_axis=self.theta - 1
                #
                self.x0 = self.x
                self.y0 = self.y
                self.num_before_reset_origin = 0
                self.I_max = mean_I_short
                logging.info(f"Reset origin at {self.x}, {self.y}, I_max= {self.I_max}")
        self.I_max = max(self.COEF_I_DECAY * self.I_max, self._I_meaningful)
        self.num_before_reset_origin += 1
        self.pts_r.append(self.r)

        # >>> update d value
        mean_I_long = self.mean(self.pts_I, self._SPIRAL_RESOLUTION)
        uds = np.arctan(1 - mean_I_long / self._I_meaningful) / (np.pi / 2)
        self.d = self.d * (1 + 0.0002 * uds)
        self.pts_d.append(self.d)

        # >>> update x, y value
        # x = self.x0 + self.r * np.cos(self.n_iter * self.delta_theta)
        # y = self.y0 + self.r * np.sin(self.n_iter * self.delta_theta)
        x = self.x0 + self.r * np.cos(self.theta)
        y = self.y0 + self.r * np.sin(self.theta)
        self.x, self.y = self.bounded((x, y))
        self.pts_x.append(self.x)
        self.pts_y.append(self.y)

        # >>> update theta value
        self.delta_theta = 2 * np.pi / self._SPIRAL_RESOLUTION
        self.theta += self.delta_theta

    def step_x0y0(self):
        #
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
            # print(mean_dx, mean_dy, ellipcity, r)
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

    def step(self):
        if self.num_before_reset_origin < self.SINGLE_SPIRAL_SPAN * self._SPIRAL_RESOLUTION:
            self.step_rdxy()
        else:
            return False
        #
        self.I = self.callback_function((self.x, self.y))
        self.pts_I.append(self.I)
        #
        # logging.info(f"step {self.n_iter}, x= {self.x}, y= {self.y}, r= {self.r}, I= {self.I}")
        self.step_x0y0()
        #
        self.n_iter += 1
        return True

    def maximize(self,function,x0,bounds,options):
        self.callback_function=function
        self.x, self.y = x0
        self.x0, self.y0 = x0
        self.bounds=bounds
        self.load_options(options)
        self.I_max = self.callback_function((self.x, self.y))
        #
        with tqdm.tqdm(total=self.SPIRAL_RESOLUTION*self.SPIRAL_SPAN) as pbar:
            while self.n_iter < self.SPIRAL_RESOLUTION*self.SPIRAL_SPAN:
                if self.step():
                    pbar.update(1)
                else:
                    break
        #
        return (self.x0, self.y0)





if __name__ == "__main__":
    # plot gaussian2d distribution
    x = np.linspace(-20, 20, 100)
    y = np.linspace(-20, 20, 100)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros(X.shape)
    mu = np.array([8, 15])
    cov = covariance_matrix(4, 0.2, np.pi / 3)
    #
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Z[i, j] = gaussian2d(X[i, j], Y[i, j], mu, cov)

    plt.contourf(X, Y, Z)

    sp = SpiralPath()
    callback_function = lambda xy: gaussian2d(xy[0], xy[1], mu, cov)
    sp.maximize(callback_function,x0=(0,0),bounds=[(-20,20),(-20,20)])

    plt.plot(sp.pts_x, sp.pts_y, "r")
    plt.plot(sp.pts_x0, sp.pts_y0, "b")
    plt.axis("equal")
    plt.figure()
    plt.plot(sp.pts_ellipcity)
    plt.show()
