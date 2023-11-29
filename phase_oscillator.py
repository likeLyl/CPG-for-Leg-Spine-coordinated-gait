# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : phase_oscillator.py
# Date      : 27-Nov-2023
# Version:  : 
# ----------------------------------------------------------------
from differential_funs import DifferentialEq

class oscillator():
    def __init__(self, step_size, ar, ax, R_st, R_sw, X, r_0, x_0):
        self.amp_st = DifferentialEq(ar, R_st, step_size, r_0)        # stance phase amplitude
        self.amp_sw = DifferentialEq(ar, R_sw, step_size, r_0)        # swing phase amplitude
        self.offset = DifferentialEq(ax, X, step_size, x_0)           # offset
        self.offset_out = 0

    def update_r(self):
        st_pos = self.amp_st.update_amp_3()        # joint position in stance phase
        sw_pos = self.amp_sw.update_amp_3()        # joint position in swing phase
        return st_pos, sw_pos

    def update_x(self):
        self.offset_out = self.offset.update_amp_3()
        return self.offset_out
