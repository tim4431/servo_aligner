import numpy as np
import matplotlib.pyplot as plt
import time
import logging
from collections import defaultdict

logging.basicConfig(
    level=logging.INFO,  # Set the logging level to DEBUG
    # level= logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

from servodriver import Servoset
from servo_util import compose_para
from optimize import fiber_coupling
from callback_functions import make_callback_func, intensity_adc
from datastore import DataStore, resolve_under_data
from config import (SERVER, SERVO_CHANNEL_LIST, COUPLING_VECTORS, JACOBIAN,
                    MASKS, knob_mask)

servos = Servoset(board_id=SERVER["board_id"],servo_channel_list=SERVO_CHANNEL_LIST)
servos.de_hysterisis=True
servos.torques_enable()
# servos.home()

# Objective: photodiode intensity over the ADC (see callback_functions.OBJECTIVES).
callback_func = make_callback_func(servos, intensity_adc)


def cord_pm_offset(N,normd,i):
    # choose a position in offset to be +-30, each sample 5 times
    j = i%4
    pm = 1 if i//4%2==0 else -1
    offset = np.zeros(4)
    offset[j] = pm*normd
    return offset

def random_norm_offset(N,normd):
    # offset with arbitrary direction with norm=normd
    offset = np.random.randn(4)
    offset = offset/np.linalg.norm(offset)*normd
    return offset

def lin_comb_offset(N,normd, vecs,i):
    vecs_normed = [v/np.linalg.norm(v) for v in vecs]
    distrib = np.random.rand(len(vecs))
    distrib = distrib/np.linalg.norm(distrib)
    vec_norm_distrib = np.sum([distrib[i]*vecs_normed[i] for i in range(len(vecs))],axis=0)
    offset = vec_norm_distrib*normd
    return offset

def load_assumed_jac(path):
    """Load a previously-fit Jacobian for extrapolation bootstrapping.

    Returns ``(jac, jac_x0)``. With ``path=None`` returns ``(None, None)`` so the
    calibration starts from the plain origin instead of a prior Jacobian.
    """
    if path is None:
        return None, None
    d = np.load(resolve_under_data(path), allow_pickle=True)
    jac = np.array(d['jac'])
    jac_x0 = np.array(d['x0']) if 'x0' in d else None
    return jac, jac_x0


# Bootstrap from a previously-fit Jacobian (extrapolation): jacobian.assume_path
# in calibration.yaml is an .npz with a 'jac' (and optional 'x0') array, or null
# to start from the plain origin.
JAC_ASSUME_PATH = JACOBIAN.get("assume_path")
jac_assume, jac_x0 = load_assumed_jac(JAC_ASSUME_PATH)

cf00 = lambda para: callback_func(para, pos_mask=MASKS["POS_ALL"],zero=np.array([0,0,0,0,0,0,0,0],dtype=float))
print(cf00([0,0,0,0,0,0,0,0]))

#
#ss
N=1
normd = JACOBIAN.get("normd", 20)
offset_type = 'zero' # 'pm' or 'rand' or 'lin' or 'zero'
MASTER = "B"  # A=upper path, B=lower path; per-stage knobs optimize the slave path
SLAVE = "A" if MASTER == "B" else "B"

# Only the pm/rand/lin probe modes accumulate a {offset: [(zero, I), ...]} dataset.
# Set up its run folder + dataset file once (warns if the folder already exists).
COLLECT = offset_type in ['pm', 'rand', 'lin']
if COLLECT:
    jac_store = DataStore(JACOBIAN.get("output_subdir", "jacobian"))
    ds_name = 'jacobian_{:s}_{:d}'.format(offset_type, normd)
    if not jac_store.exists(ds_name, ".npy"):
        jac_store.save_npy(ds_name, defaultdict(list))
#
for i in range(N):
    if COLLECT:
        dataset = jac_store.load_npy(ds_name).item()
        # print(dataset)

    zero = np.array([0,0,0,0,0,0,0,0],dtype=float)
    # zero = np.array([3.0, 58.0, 11.0, -85.0, 0.0, 0.0, 0.0, 0.0],dtype=float)
    cf0 = lambda para: callback_func(para, pos_mask=MASKS["POS_ALL"],zero=zero)
    # print(cf0([0,0,0,0,0,0,0,0]))
    #
    offset_mask = knob_mask(MASTER, "POS_ALL")

    if offset_type == 'pm':
        offset = cord_pm_offset(N,normd,i)
    elif offset_type == 'rand':
        offset = random_norm_offset(N,normd)
    elif offset_type == 'lin':
        # coupling directions per master path live in calibration.yaml.
        vecs = [np.array(v) for v in COUPLING_VECTORS[MASTER]]
        offset = lin_comb_offset(N,normd,vecs,i)
    elif offset_type == 'zero':
        offset = np.zeros(np.sum(offset_mask))
    elif offset_type == 'spec':
        offset = np.array([3,0,0,0])
    #
    zero = compose_para(para=offset,pos_mask = offset_mask, zero=zero,jac=jac_assume,jac_master_mask=offset_mask,jac_x0=jac_x0)
    logging.info(f"Offset = {offset}, Zero = {zero}")
    #
    #
    try:
        logging.info(f"Start optimization with zero = {zero}")
        # The fiber-coupling step sequence (X_Y -> X_XDOT -> Y_YDOT -> joint
        # L-BFGS-B over POS_ALL) now lives in optimize.fiber_coupling.
        zero, I = fiber_coupling(servos, callback_func, zero, path=SLAVE)
        print(I)
        #
        if COLLECT:
            dataset[tuple(offset)].append((list(zero),I))
            # print(dataset)
            jac_store.save_npy(ds_name, dataset)
        print("i=",i)

    except Exception as e:
        servos.close()
        logging.error(f"Error in iterative_optimize: {e}")