import numpy as np
from pts_iterator import pts_iterator
import logging
from servo_util import format_para, nraddr

#
spiral_params = {
    'I_meaningful': 0.005,
    'D': 2.4,
    'SPIRAL_RESOLUTION': 14,
    'SPIRAL_SPAN': 6,
    'SINGLE_SPIRAL_SPAN': 3.5,
    'N_LOOPS_BEFORE_RESET_ORIGIN': 0.5,
    'MAX_X0Y0_DISPLACEMENT': 10,
    'COEF_I_RESET_ORIGIN': 1.4,
    'alpha': 0.03,
    'COEF_I_DECAY': 0.995
}
BFGS_params = {"disp": True, "maxiter": 10,  "eps": 5}
#


def step_optimize(servos,
                  callback_func,
                  pos_mask=None,
                  p0=None,
                  zero=None,
                  method="spiral",
                  bounds_single = (-100,100),
                  )->np.ndarray:
    #
    if method == 'L-BFGS-B':
        # servos.set_precision(1)
        options = BFGS_params
    elif method == 'spiral':
        # servos.set_precision(5)
        options = spiral_params
    else:
        raise ValueError(f"unknown method: {method}")
    #
    N_var = np.sum(pos_mask)
    if p0 is None:
        p0 = np.zeros(N_var)
    bounds = [bounds_single for i in range(N_var)]
    cf = lambda x: callback_func(x,pos_mask,zero=zero)
    Istart = cf(p0)[1]
    logging.info(f"Start position: {format_para(p0)}, start I: {Istart}")
    #
    para, Ibst = pts_iterator(N_var=N_var,callback_func=cf, p0=p0, bounds = bounds, options=options, method = method)
    #
    para = list(para)
    Inow = cf(para)[1]
    logging.info(f"Best position: {format_para(para)}, now I: {Inow}")
    #
    if Inow/Ibst > 0.7:
        logging.info(f"New Origin set to be {format_para(para)}")
        zero_fullnd = nraddr(zero,para,pos_mask)
    else:
        logging.info("The intensity is not high enough, operation cancelled.")
        zero_fullnd = np.array(zero)
    #
    logging.info(f"Zero = {zero_fullnd}")
    return zero_fullnd

