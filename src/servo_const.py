"""Channel grouping masks and lookups.

The masks select which channel indices form each knob group (e.g.
``A_X_XDOT_MASK = [1,0,1,0,0,0,0,0]``). They depend on the optics setup - which
servo drives which knob - so the values live in ``calibration.yaml`` (the
``masks:`` section) and are loaded here via :mod:`config`. The constant names
below are kept stable so the rest of the code can keep importing them.
"""

from config import MASKS

A_X_XDOT_MASK = MASKS["A_X_XDOT"]
A_Y_YDOT_MASK = MASKS["A_Y_YDOT"]
A_X_Y_MASK = MASKS["A_X_Y"]
A_XDOT_YDOT_MASK = MASKS["A_XDOT_YDOT"]
A_POS_ALL_MASK = MASKS["A_POS_ALL"]
#
B_X_XDOT_MASK = MASKS["B_X_XDOT"]
B_Y_YDOT_MASK = MASKS["B_Y_YDOT"]
B_X_Y_MASK = MASKS["B_X_Y"]
B_XDOT_YDOT_MASK = MASKS["B_XDOT_YDOT"]
B_POS_ALL_MASK = MASKS["B_POS_ALL"]
#
POS_ALL_MASK = MASKS["POS_ALL"]
#
def posmask2str(posmask):
    # if posmask is A_X_XDOT_MASK, then return "A_X_XDOT", etc.
    posmask_list = [A_X_XDOT_MASK, A_Y_YDOT_MASK, A_X_Y_MASK, A_XDOT_YDOT_MASK, A_POS_ALL_MASK, B_X_XDOT_MASK, B_Y_YDOT_MASK, B_X_Y_MASK, B_XDOT_YDOT_MASK, B_POS_ALL_MASK]
    posmask_str_list = ["A_X_XDOT", "A_Y_YDOT", "A_X_Y", "A_XDOT_YDOT", "A_POS_ALL", "B_X_XDOT", "B_Y_YDOT", "B_X_Y", "B_XDOT_YDOT", "B_POS_ALL"]
    for i in range(len(posmask_list)):
        if posmask == posmask_list[i]:
            return posmask_str_list[i]
    return None
