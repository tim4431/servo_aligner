"""Raster scans of the objective over one or two knob directions.

``motor_2d_scan`` sweeps a 2D grid over a knob pair (boustrophedon order via
``servo_util.create_zigzag_X``, minimising motor travel) and returns the
``(X, Y, Z)`` intensity map; ``clip_scan.py`` fits the beam-clip center to it.
``motor_1d_scan`` is the 1D line-scan variant. Both take the optimizer-style
``callback_func`` (move, then read the objective) and an optional
``accept_func`` that skips grid points outside the region of interest.

An error mid-scan is logged and the partial ``Z`` is returned; either way the
servos are sent home afterwards.
"""

import numpy as np
import tqdm
import matplotlib.pyplot as plt
import logging
from servo_util import create_zigzag_X

def motor_1d_scan(N_pts, scan_range,scan_vec, servo, callback_func, accept_func=None):
    L = np.linspace(-scan_range,scan_range,N_pts)
    X = L * scan_vec[0]
    Y = L * scan_vec[1]
    Z = np.zeros_like(L)
    #
    z0 = callback_func([0,0])
    logging.info(f"z0: {z0}")

    #
    try:
        with tqdm.tqdm(total=len(L)) as pbar:
            for i in range(len(L)):
                x,y = X[i], Y[i]
                if (accept_func is None) or (accept_func([x,y])):
                    para,z = callback_func(para=[x,y])
                    Z[i] = z
                pbar.update(1)
    except Exception as e:
        logging.error(f"Error in motor_1d_scan: {e}")
    finally:
        servo.home()
    #
    plt.plot(L,Z)
    plt.xlabel("L")
    plt.ylabel("Z")
    return L,Z



def motor_2d_scan(N_pts, scan_range, servos, callback_func, accept_func=None):
    if isinstance(N_pts, tuple):
        N_pts_x, N_pts_y = N_pts
        if isinstance(scan_range, tuple):
            scan_range_x, scan_range_y = scan_range
        else:
            scan_range_x, scan_range_y = scan_range, scan_range
        Xs = np.linspace(-scan_range_x,scan_range_x,N_pts_x)
        Ys = np.linspace(-scan_range_y,scan_range_y,N_pts_y)
    else:
        # create the grid
        Xs = np.linspace(-scan_range,scan_range,N_pts)
        Ys = np.linspace(-scan_range,scan_range,N_pts)
    #
    X,Y = np.meshgrid(Xs,Ys)
    Z = np.zeros_like(X)
    X_zig, index_map = create_zigzag_X(X)
    z0 = callback_func([0,0])
    logging.info(f"z0: {z0}")

    #
    try:
        with tqdm.tqdm(total=len(Xs)*len(Ys)) as pbar:
            for i in range(len(Ys)):
                for j in range(len(Xs)):
                    x,y = X_zig[i,j], Y[i,j]
                    idx = index_map[i,j] # original index of X
                    idx_i, idx_j = np.unravel_index(idx, X.shape) # r[idx_i, idx_j] == r_zig[i,j]
                    #
                    if (accept_func is None) or (accept_func([x,y])):
                        para,z = callback_func(para=[x,y])
                        Z[idx_i,idx_j] = z
                    pbar.update(1)

    except Exception as e:
        logging.error(f"Error in motor_2d_scan: {e}")
    finally:
        servos.home()
    #
    plt.matshow(Z,extent=[np.min(X),np.max(X),np.min(Y),np.max(Y)],origin="lower")
    plt.colorbar()
    logging.info(f"Z range: {np.min(Z)} .. {np.max(Z)}")
    return X,Y,Z
