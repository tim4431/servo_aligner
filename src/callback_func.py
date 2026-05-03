from servo_util import compose_para, r2nd
import numpy as np

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
    
# cf0 = lambda para: callback_func(para, pos_mask=POS_ALL_MASK)
# cf0([0,0,0,0,0,0,0,0])