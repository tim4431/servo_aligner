import numpy as np
import matplotlib.pyplot as plt
import time
from scservo_sdk import *                    # Uses SCServo SDK library
from pts_iterator import pts_iterator
import logging

logging.basicConfig(
    level=logging.INFO,  # Set the logging level to DEBUG
    # level= logging.DEBUG,|
    format='%(asctime)s - %(levelname)s - %(message)s'
)

from servodriver import Servoset
from servo_util import create_zigzag_X, format_para,a2p,r2nd,r2nr,ndmodr,nrselr,nrmodr,nraddr,compose_para
from fit_gaussian import gaussian_2d,gaussian_2d_smooth_heaviside,fit_and_plot,fit_gaussian_2d,fit_gaussian_2d_smooth_heaviside,fit_and_plot_smooth_heaviside,popt_get_mu_cov
from motor_scan import motor_2d_scan
from servo_const import A_X_XDOT_MASK,A_Y_YDOT_MASK,A_X_Y_MASK,A_XDOT_YDOT_MASK,A_POS_ALL_MASK,B_X_XDOT_MASK,B_Y_YDOT_MASK,B_X_Y_MASK,B_XDOT_YDOT_MASK,B_POS_ALL_MASK,POS_ALL_MASK, posmask2str
from pd import MCP3424_fiber
from step_optimize import step_optimize


servos = Servoset(board_id=0,servo_channel_list=[0,1,2,3,4,5,6,7])
servos.de_hysterisis=False
servos.torques_enable()

FOLDER = "/home/rydpiservo/servodata/servorecover5/"

def callback_func(para,
                  pos_mask,
                  zero=None,
                  jac=None,
                  jac_master_mask=None,
                  debug=False,
                  **kwargs):

    para_nr_move = compose_para(para, pos_mask, zero, jac, jac_master_mask,debug=debug,**kwargs)
    #
    if not debug:
        servos.set_angle(list(para_nr_move))
        #
        data_cache = []
        for m in range(2):
            # data = ADS1115_fiber.value
            data = MCP3424_fiber.convert_and_read()
            data_cache.append(data)
        z = float(np.mean(np.array(data)))
        # print(para,z)
        return tuple(para),z



def accept_func_linearxy(xy, slope, b, tol):
    x,y = xy[0],xy[1]
    return np.abs(y-x*slope-b)<tol

accept_func_AXXDOT = lambda xy: accept_func_linearxy(xy,1.35,0,80)
accept_func_AYYDOT = lambda xy: accept_func_linearxy(xy,-1.36,5,80)
accept_func_BXXDOT = lambda xy: accept_func_linearxy(xy,-0.67,0,80)
accept_func_BYYDOT = lambda xy: accept_func_linearxy(xy,-0.67,5,80)

def posmask2acceptfunc(posmask):
    if posmask == A_X_XDOT_MASK:
        return accept_func_AXXDOT
    elif posmask == A_Y_YDOT_MASK:
        return accept_func_AYYDOT
    elif posmask == B_X_XDOT_MASK:
        return accept_func_BXXDOT
    elif posmask == B_Y_YDOT_MASK:
        return accept_func_BYYDOT
    else:
        raise ValueError("posmask not recognized")
        return None

def scan_and_analyze(zero,N_pts,ITER_NUM,POS_MASK,SCAN_RANGE,enable_accfunc=True,plot_type=0):
    # PERFORM SCAN
    time.sleep(1)
    #,S
    logging.info("SCANNING: {} {}".format(POS_MASK,ITER_NUM))
    cf = lambda para: callback_func(para,pos_mask=POS_MASK,zero=zero)
    scan_start_time = time.time()
    accept_func = posmask2acceptfunc(POS_MASK) if enable_accfunc else None
    X,Y,Z = motor_2d_scan(N_pts,SCAN_RANGE,servos,cf,accept_func=accept_func)
    scan_stop_time = time.time()
    posmaskstr = posmask2str(POS_MASK)
    fileName="clip_{}_{}".format(posmaskstr,ITER_NUM)
    np.savez("{}{}.npz".format(FOLDER,fileName),X=X,Y=Y,Z=Z,posmaskstr = posmaskstr,ITER_NUM=ITER_NUM,scan_start_time=scan_start_time,scan_stop_time=scan_stop_time)
    Z_norm = Z/np.max(Z)
    #
    # XXDOT/YYDOT plot
    if plot_type == 0:
        # plot
        fig,ax = plt.subplots(1,4,figsize=(20,5))

        # ax0. accept_func
        ax0 = ax[0]
        Zacc = np.zeros_like(X)
        if isinstance(N_pts,int):
            N_pts_x,N_pts_y = N_pts,N_pts
        elif isinstance(N_pts,tuple):
            N_pts_x,N_pts_y = N_pts

        accept_func = posmask2acceptfunc(POS_MASK)
        xy = np.array([X, Y])  # Combine X and Y arrays
        Zacc = accept_func(xy).astype(int)  # Apply the acceptance function and convert to int
        ax0.imshow(Zacc*0.2+Z_norm, extent=[X.min(), X.max(), Y.min(), Y.max()],origin='lower')

        # ax1. fit_gaussian_2d
        ax1 = ax[1]
        popt=fit_and_plot_smooth_heaviside(X,Y,Z,ax=ax1)
        mu,cov = popt_get_mu_cov(popt)
        ax1.scatter(mu[0],mu[1],marker='x',color='black')
        eigvals, eigvecs = np.linalg.eig(cov)
        sigmas= np.sqrt(eigvals)
        print(popt)
        print(sigmas)


        # ax2. central point magnify
        ax2 = ax[2]
        ax2.imshow(Z_norm, extent=[X.min(), X.max(), Y.min(), Y.max()],origin='lower')
        ax2.scatter(mu[0],mu[1],marker='x',color='black')
        ax2.set_xlim([mu[0]-100,mu[0]+100])
        ax2.set_ylim([mu[1]-100,mu[1]+100])


        # ax3, row sum
        ax3 = ax[3]
        Z_row = np.sum(Z,axis=0)
        ax3.plot(X[0,:],Z_row)
        #
        # suptitle
        fig.suptitle("{}, mu = ({:.3f},{:.3f}), sigmas = ({:.3f},{:.3f})".format(fileName,mu[0],mu[1],sigmas[0],sigmas[1]))
        plt.savefig("{}{}.png".format(FOLDER,fileName),dpi=300,bbox_inches='tight')
        #
    #
    elif plot_type == 1:
        # plot
        fig,ax = plt.subplots(figsize=(10,5))

        # ax1. fit_gaussian_2d
        popt=fit_and_plot_smooth_heaviside(X,Y,Z,ax=ax)
        mu,cov = popt_get_mu_cov(popt)
        ax.scatter(mu[0],mu[1],marker='x',color='black')
        eigvals, eigvecs = np.linalg.eig(cov)
        sigmas= np.sqrt(eigvals)
        print(popt)
        print(sigmas)
        ax.set_title("{}, mu = ({:.3f},{:.3f}), sigmas = ({:.3f},{:.3f})".format(fileName,mu[0],mu[1],sigmas[0],sigmas[1]))
        plt.savefig("{}{}.png".format(FOLDER,fileName),dpi=300,bbox_inches='tight')

    #
    cf(list(mu))
    time.sleep(1)
    logging.info("going to mu: {}".format(mu))
    _,I = cf(list(mu))
    logging.info("I: {}".format(I))
    #
    if I>I_meaningful:
        logging.info("calculating new zero point")
        zero_new = nraddr(zero,np.array(list(mu)),POS_MASK)
        logging.info("zero_new: {}".format(zero_new))
        # servos.set_zero()
    else:
        logging.info("I too small, not setting zero point")
        zero_new = zero
    np.savez("{}{}_popt.npz".format(FOLDER,fileName),popt=popt,mu=mu,cov=cov,sigmas=sigmas,zero=zero_new)
    #
    logging.info("DONE SCANNING: {} {}".format(POS_MASK,ITER_NUM))
    return zero_new



if __name__ == "__main__":
    I_meaningful = 0.1
    zero = np.array([0,0,0,0,0,0,0,0],dtype=float)
    # fileName = "clip_A_Y_YDOT_3"
    # d = np.load("{}{}_popt.npz".format(FOLDER,fileName))
    # zero = d['zero']
    # print(zero)
    cf0 = lambda para: callback_func(para, pos_mask=POS_ALL_MASK,zero=zero)
    print(cf0([0,0,0,0,0,0,0,0]))
    #
    for ITER_NUM in range(6,7):
        for POS_MASK in [A_X_XDOT_MASK,A_Y_YDOT_MASK]:
        # for POS_MASK in [A_Y_YDOT_MASK]:
            N_pts = 50
            SCAN_RANGE = 500 if POS_MASK==A_X_XDOT_MASK else 800
            zero = scan_and_analyze(zero,N_pts,ITER_NUM,POS_MASK,SCAN_RANGE,plot_type=0)
        #
        N_pts = 15
        POS_MASK = A_X_Y_MASK
        SCAN_RANGE = 30
        zero = scan_and_analyze(zero,N_pts,ITER_NUM,POS_MASK,SCAN_RANGE,enable_accfunc=False,plot_type=1)