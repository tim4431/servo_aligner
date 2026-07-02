import numpy as np
import matplotlib.pyplot as plt
import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)

import _bootstrap  # noqa: F401 -- prepend ../src on sys.path for the library imports below
from servodriver import Servoset
from servo_util import nraddr
from fit_gaussian import fit_and_plot_smooth_heaviside, popt_get_mu_cov
from motor_scan import motor_2d_scan
from callback_functions import make_callback_func, intensity_adc
from datastore import DataStore
from config import (
    SERVER,
    SERVO_CHANNEL_LIST,
    ACCEPT_FUNCTIONS,
    CLIP_SCAN,
    MASKS,
    knob_mask,
    posmask2str,
)

servos = Servoset(board_id=SERVER["board_id"], servo_channel_list=SERVO_CHANNEL_LIST)
servos.de_hysterisis = False
servos.set_torque(True)

# Run folder for scan results, under the data/ root (warns if it already exists).
STORE = DataStore(CLIP_SCAN.get("output_subdir", "clip_scan"))
# Minimum fitted intensity to accept a new zero point (clip_scan in calibration.yaml).
I_meaningful = CLIP_SCAN.get("I_meaningful", 0.1)


# Objective: photodiode intensity over the ADC (see callback_functions.OBJECTIVES).
callback_func = make_callback_func(servos, intensity_adc)


def accept_func_linearxy(xy, slope, b, tol):
    x, y = xy[0], xy[1]
    return np.abs(y - x * slope - b) < tol


def posmask2acceptfunc(posmask):
    # accept-function coefficients live in calibration.yaml, keyed by mask name.
    name = posmask2str(posmask)
    params = ACCEPT_FUNCTIONS.get(name)
    if params is None:
        raise ValueError("No accept function configured for posmask {}".format(name))
    return lambda xy: accept_func_linearxy(
        xy, params["slope"], params["b"], params["tol"]
    )


def scan_and_analyze(
    zero, N_pts, ITER_NUM, POS_MASK, SCAN_RANGE, enable_accfunc=True, plot_type=0
):
    # PERFORM SCAN
    time.sleep(1)
    logging.info("SCANNING: {} {}".format(POS_MASK, ITER_NUM))
    cf = lambda para: callback_func(para, pos_mask=POS_MASK, zero=zero)
    scan_start_time = time.time()
    accept_func = posmask2acceptfunc(POS_MASK) if enable_accfunc else None
    X, Y, Z = motor_2d_scan(N_pts, SCAN_RANGE, servos, cf, accept_func=accept_func)
    scan_stop_time = time.time()
    posmaskstr = posmask2str(POS_MASK)
    fileName = "clip_{}_{}".format(posmaskstr, ITER_NUM)
    STORE.save_npz(
        fileName,
        X=X,
        Y=Y,
        Z=Z,
        posmaskstr=posmaskstr,
        ITER_NUM=ITER_NUM,
        scan_start_time=scan_start_time,
        scan_stop_time=scan_stop_time,
    )
    Z_norm = Z / np.max(Z)
    #
    # XXDOT/YYDOT plot
    if plot_type == 0:
        fig, ax = plt.subplots(1, 4, figsize=(20, 5))

        # ax0. accept_func region overlaid on the normalized scan
        ax0 = ax[0]
        accept_func = posmask2acceptfunc(POS_MASK)
        xy = np.array([X, Y])
        Zacc = accept_func(xy).astype(int)
        ax0.imshow(
            Zacc * 0.2 + Z_norm,
            extent=[X.min(), X.max(), Y.min(), Y.max()],
            origin="lower",
        )

        # ax1. fit_gaussian_2d
        ax1 = ax[1]
        popt = fit_and_plot_smooth_heaviside(X, Y, Z, ax=ax1)
        mu, cov = popt_get_mu_cov(popt)
        ax1.scatter(mu[0], mu[1], marker="x", color="black")
        eigvals, eigvecs = np.linalg.eig(cov)
        sigmas = np.sqrt(eigvals)
        logging.info("fit popt: {}, sigmas: {}".format(popt, sigmas))

        # ax2. central point magnify
        ax2 = ax[2]
        ax2.imshow(Z_norm, extent=[X.min(), X.max(), Y.min(), Y.max()], origin="lower")
        ax2.scatter(mu[0], mu[1], marker="x", color="black")
        ax2.set_xlim([mu[0] - 100, mu[0] + 100])
        ax2.set_ylim([mu[1] - 100, mu[1] + 100])

        # ax3, row sum
        ax3 = ax[3]
        Z_row = np.sum(Z, axis=0)
        ax3.plot(X[0, :], Z_row)
        #
        # suptitle
        fig.suptitle(
            "{}, mu = ({:.3f},{:.3f}), sigmas = ({:.3f},{:.3f})".format(
                fileName, mu[0], mu[1], sigmas[0], sigmas[1]
            )
        )
        STORE.save_fig(fileName, dpi=300, bbox_inches="tight")
        #
    #
    elif plot_type == 1:
        fig, ax = plt.subplots(figsize=(10, 5))

        # fit_gaussian_2d only
        popt = fit_and_plot_smooth_heaviside(X, Y, Z, ax=ax)
        mu, cov = popt_get_mu_cov(popt)
        ax.scatter(mu[0], mu[1], marker="x", color="black")
        eigvals, eigvecs = np.linalg.eig(cov)
        sigmas = np.sqrt(eigvals)
        logging.info("fit popt: {}, sigmas: {}".format(popt, sigmas))
        ax.set_title(
            "{}, mu = ({:.3f},{:.3f}), sigmas = ({:.3f},{:.3f})".format(
                fileName, mu[0], mu[1], sigmas[0], sigmas[1]
            )
        )
        STORE.save_fig(fileName, dpi=300, bbox_inches="tight")

    # Move to the fitted center, let the mount settle, then re-read there.
    cf(list(mu))
    time.sleep(1)
    logging.info("going to mu: {}".format(mu))
    _, I = cf(list(mu))
    logging.info("I: {}".format(I))
    #
    if I > I_meaningful:
        logging.info("calculating new zero point")
        zero_new = nraddr(zero, np.array(list(mu)), POS_MASK)
        logging.info("zero_new: {}".format(zero_new))
    else:
        logging.info("I too small, not setting zero point")
        zero_new = zero
    STORE.save_npz(
        fileName + "_popt",
        popt=popt,
        mu=mu,
        cov=cov,
        sigmas=sigmas,
        zero=zero_new,
    )
    #
    logging.info("DONE SCANNING: {} {}".format(POS_MASK, ITER_NUM))
    return zero_new


if __name__ == "__main__":
    # To resume from a previous scan's fitted zero, load it from the store:
    #   zero = STORE.load_npz("clip_A_Y_YDOT_3_popt")["zero"]
    zero = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=float)
    cf0 = lambda para: callback_func(para, pos_mask=MASKS["POS_ALL"], zero=zero)
    logging.info("Objective at origin: {}".format(cf0([0, 0, 0, 0, 0, 0, 0, 0])))
    #
    for ITER_NUM in range(6, 7):
        for POS_MASK in [knob_mask("A", "X_XDOT"), knob_mask("A", "Y_YDOT")]:
            N_pts = CLIP_SCAN.get("N_pts_fine", 50)
            SCAN_RANGE = (
                CLIP_SCAN.get("scan_range_xxdot", 500)
                if POS_MASK == knob_mask("A", "X_XDOT")
                else CLIP_SCAN.get("scan_range_yydot", 800)
            )
            zero = scan_and_analyze(
                zero, N_pts, ITER_NUM, POS_MASK, SCAN_RANGE, plot_type=0
            )
        #
        N_pts = CLIP_SCAN.get("N_pts_coarse", 15)
        POS_MASK = knob_mask("A", "X_Y")
        SCAN_RANGE = CLIP_SCAN.get("scan_range_xy", 30)
        zero = scan_and_analyze(
            zero,
            N_pts,
            ITER_NUM,
            POS_MASK,
            SCAN_RANGE,
            enable_accfunc=False,
            plot_type=1,
        )
