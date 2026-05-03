import numpy as np
from fit_gaussian import (
    fit_gaussian_2d_smooth_heaviside,
    popt_get_mu_cov,
    fit_and_plot_smooth_heaviside,
)
import matplotlib.pyplot as plt

L = 16e-3
d = 1.4e-3
L1 = 0.2
L2 = 0.2 * 2.35
RANGE = 400 * (np.pi / 180)
D = 80
tList1 = np.linspace(-RANGE, RANGE, D)
tList2 = np.linspace(-RANGE, RANGE, D)
X, Y = np.meshgrid(tList1, tList2)


def theta(x, x0, w):
    # -w/2 < x < w/2, 0 otherwise
    return np.where(np.abs(x - x0) < w / 2, 1, 0)


def f(x, t):
    x1 = x + (L / 2) * np.tan(t)
    x2 = x - (L / 2) * np.tan(t)
    return theta(np.sqrt(x1**2 + x2**2), 0, d)
    # return theta(x1, 0, d)


def g(x, y, tx, ty):
    x1 = x + (L / 2) * np.tan(tx)
    x2 = x - (L / 2) * np.tan(tx)
    y1 = y + (L / 2) * np.tan(ty)
    y2 = y - (L / 2) * np.tan(ty)
    return theta(np.sqrt(x1**2 + y1**2), 0, d) * theta(np.sqrt(x2**2 + y2**2), 0, d)


def t_mirror(t):
    return 8e-3 * (t / (2 * np.pi))


def calc_data(crosstalk_matrix, zero, scan_type="xxdot"):
    Z = np.zeros((D, D))
    xZ = np.zeros((D, D))
    tZ = np.zeros((D, D))

    for i in range(D):
        for j in range(D):
            tknob1 = X[i, j]
            tknob2 = Y[i, j]
            if scan_type == "xxdot":
                tx1 = t_mirror(tknob1 + zero[0])
                tx2 = t_mirror(tknob2 + zero[2])
                tx_vec = np.array([tx1, tx2])
                cm = crosstalk_matrix(tknob1, tknob2)
                ty_vec = np.dot(cm, tx_vec)
                ty1 = t_mirror(zero[1]) + ty_vec[0]
                ty2 = t_mirror(zero[3]) + ty_vec[1]
            elif scan_type == "yydot":
                ty1 = t_mirror(tknob1 + zero[1])
                ty2 = t_mirror(tknob2 + zero[3])
                ty_vec = np.array([ty1, ty2])
                cm = crosstalk_matrix(tknob1, tknob2)
                # tx_vec = np.dot(np.linalg.inv(cm), ty_vec)
                tx_vec = np.dot(cm, ty_vec)
                tx1 = t_mirror(zero[0]) + tx_vec[0]
                tx2 = t_mirror(zero[2]) + tx_vec[1]
            #
            tx = tx1 + tx2
            ty = ty1 + ty2
            x = -L1 * np.tan(tx1 * 2) + L2 * np.tan(tx * 2)
            y = -L1 * np.tan(ty1 * 2) + L2 * np.tan(ty * 2)
            transmission = g(x, y, tx, ty)
            Z[i, j] = transmission
            xZ[i, j] = x*transmission
            tZ[i, j] = tx*transmission
    return Z,xZ,tZ


# crosstalk_matrix = lambda tknob1, tknob2: np.array([[0.1, 0.2], [0.4, 0.2]])
crosstalk_matrix = lambda tknob1, tknob2: np.array([[0.1,0], [0, 0.3]])
crosstalk_matrix_rot = lambda tknob1, tknob2: np.array(
    [[0.1+tknob1 / 6,0], [0, 0.3 + tknob1 / 10]]
)


zero=np.array([0, 0, 0, 0],dtype=float)
Z = calc_data(crosstalk_matrix=crosstalk_matrix, zero=zero, scan_type="xxdot")
plt.show()