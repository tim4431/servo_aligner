A_X_XDOT_MASK = [1,0,1,0,0,0,0,0]
A_Y_YDOT_MASK = [0,1,0,1,0,0,0,0]
A_X_Y_MASK = [1,1,0,0,0,0,0,0]
A_XDOT_YDOT_MASK = [0,0,1,1,0,0,0,0]
A_POS_ALL_MASK = [1,1,1,1,0,0,0,0]
#
B_X_XDOT_MASK = [0,0,0,0,1,0,1,0]
B_Y_YDOT_MASK = [0,0,0,0,0,1,0,1]
B_X_Y_MASK = [0,0,0,0,0,0,1,1]
B_XDOT_YDOT_MASK = [0,0,0,0,1,1,0,0]
B_POS_ALL_MASK = [0,0,0,0,1,1,1,1]
#
POS_ALL_MASK = [1,1,1,1,1,1,1,1]
#
def posmask2str(posmask):
    # if posmask is A_X_XDOT_MASK, then return "A_X_XDOT", etc.
    posmask_list = [A_X_XDOT_MASK, A_Y_YDOT_MASK, A_X_Y_MASK, A_XDOT_YDOT_MASK, A_POS_ALL_MASK, B_X_XDOT_MASK, B_Y_YDOT_MASK, B_X_Y_MASK, B_XDOT_YDOT_MASK, B_POS_ALL_MASK]
    posmask_str_list = ["A_X_XDOT", "A_Y_YDOT", "A_X_Y", "A_XDOT_YDOT", "A_POS_ALL", "B_X_XDOT", "B_Y_YDOT", "B_X_Y", "B_XDOT_YDOT", "B_POS_ALL"]
    for i in range(len(posmask_list)):
        if posmask == posmask_list[i]:
            return posmask_str_list[i]
    return None