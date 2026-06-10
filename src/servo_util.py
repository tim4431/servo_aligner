import numpy as np

def create_zigzag_X(X):
    X_zigzag = np.copy(X)  # Create a copX of X to modifX
    index_map = np.zeros_like(X, dtype=int)  # To track the original indices, elements in X_zigzag and their original index in X

    # Alter rows in a zigzag pattern
    for i in range(X_zigzag.shape[0]):
        if i % 2 == 0:
            # Even rows: keep as is
            X_zigzag[i, :] = X[i, :]
            index_map[i, :] = np.arange(X_zigzag.shape[1]) + i * X_zigzag.shape[1]
        else:
            # Odd rows: reverse the order
            X_zigzag[i, :] = np.flip(X[i, :])
            index_map[i, :] = np.flip(np.arange(X_zigzag.shape[1])) + i * X_zigzag.shape[1]
    return X_zigzag, index_map

def a2p(angle):
    return int(angle*(4096/360)+2048)

def r2nd(r,r_mask=None)->np.ndarray:
    if r_mask is None:
        r_mask = np.ones_like(r,dtype=np.bool_)
    # arbitrary r embedded in the mask
    r = list(r)
    assert np.sum(r_mask) == len(r), ValueError("r2nd: r_mask should have the same 1 as len(r)")
    pos = np.ones_like(r_mask,dtype=np.int_)*a2p(0)
    for i in range(len(r_mask)):
        if r_mask[i]:
            pos[i] = a2p(r.pop(0))
    return pos.astype(np.int_)

def r2nr(r,r_mask=None)->np.ndarray:
    if r_mask is None:
        r_mask = np.ones_like(r,dtype=np.bool_)
    # arbitrary r embedded in the mask
    r = list(r)
    assert np.sum(r_mask) == len(r), ValueError("r2nr: r_mask should have the same 1 as len(r)")
    pos = np.zeros_like(r_mask,dtype=np.float64)
    for i in range(len(r_mask)):
        if r_mask[i]:
            pos[i] = r.pop(0)
    return pos

def nrselr(pos,r_mask)->np.ndarray:
    # arbitrary r embedded in the mask
    r = []
    for i in range(len(r_mask)):
        if r_mask[i]:
            r.append(pos[i])
    return np.array(r)

def nrmodr(r_origin,r_mod,r_mask)->np.ndarray:
    if r_mask is None:
        r_mask = np.ones_like(r_origin,dtype=np.bool_)
    # arbitrary r_mod embedded in the mask
    r_mod = list(r_mod)
    r_origin = np.array(r_origin)
    assert np.sum(r_mask) == len(r_mod), ValueError("nrmodr: r_mask should have the same length as r_mod")
    for i in range(len(r_mask)):
        if r_mask[i]:
            r_origin[i] = r_mod.pop(0)
    return r_origin

def nraddr(r_origin,r_add,r_mask=None)->np.ndarray:
    if r_mask is None:
        r_mask = np.ones_like(r_origin,dtype=np.bool_)
    # arbitrary r_add embedded in the mask
    r_add = list(r_add)
    r_origin = np.array(r_origin)
    assert np.sum(r_mask) == len(r_add), ValueError("nraddr: r_mask should have the same length as r_add")
    for i in range(len(r_mask)):
        if r_mask[i]:
            r_origin[i] += r_add.pop(0)
    return r_origin

def ndmodr(pos_origin,r_mod,r_mask)->np.ndarray:
    # arbitrary r_mod embedded in the mask
    r_mod = list(r_mod)
    pos_origin = np.array(pos_origin)
    #
    assert np.sum(r_mask) == len(r_mod), ValueError("r_mask should have the same length as r_mod")
    for i in range(len(r_mask)):
        if r_mask[i]:
            pos_origin[i] = a2p(r_mod.pop(0))
    return pos_origin.astype(np.int_)

def format_para(para):
    str_para = " ".join(["x_{:d}={:.2f}".format(i,para[i]) for i in range(len(para))])
    return str_para

def compose_para(para,
                 pos_mask,
                 zero=None,
                 jac=None,
                 jac_master_mask=None,
                 jac_master_offset=None,
                 jac_x0=None,
                 debug=False):
    # default para is zero
    if para is None:
        para = np.zeros(len(pos_mask))
    # start from zero point, step para
    if zero is None:
        zero = np.zeros(len(pos_mask))
    # dB=J(dA-jac_master_offset)
    if jac_master_offset is None:
        jac_master_offset = np.zeros(len(pos_mask))
    else:
        jac_master_offset = r2nr(jac_master_offset,jac_master_mask)
        # print("jac_master_offset",jac_master_offset)
    #
    #
    para_nr_move = nraddr(zero,para,pos_mask)
    # set slave knobs according to jac
    if jac is not None:
        assert jac_master_mask is not None, "jac_master_mask is not provided"
        dr = r2nr(para,r_mask = pos_mask) - jac_master_offset
        dr = nrselr(dr,jac_master_mask)
        d_slave_r = np.dot(jac,dr)
        if jac_x0 is not None:
            d_slave_r = d_slave_r + jac_x0
        jac_slave_mask = 1-np.array(jac_master_mask)
        para_nr_move = nraddr(para_nr_move,d_slave_r,jac_slave_mask)
        if debug:
            print(dr)
            print(d_slave_r)
            print(para_nr_move)
    return para_nr_move

if __name__=="__main__":
    a =r2nd([-5,4],[1,0,1,0])
    print(a)