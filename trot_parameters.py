# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : trot_parameters.py
# Date      : 27-Nov-2023
# Version:  : 
# ----------------------------------------------------------------
from cpg_configuration import CpgConfig
import numpy as np

# default constant variables
OSCILLATOR_NUMBERS = 16
CW_VALUE = 4
TIME_STEP = 1 / 60
CONSTANT_AR = 20
CONSTANT_AX = 20

# Default parameters
INITIAL_AMPLITUDE = [0 for i in range(OSCILLATOR_NUMBERS)]
INITIAL_OFFSET = [0 for i in range(OSCILLATOR_NUMBERS)]
INITIAL_PHASE = [0 for i in range(OSCILLATOR_NUMBERS)]

# trot gait parameters, 16 oscillators including spinal motion

# 1. phase lag
# WALK_VECTOR = [0, 0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi, -np.pi, -np.pi/2, 0, np.pi/2, -np.pi/2, 0, np.pi/2, np.pi]
TROT_VECTOR = [0, 0, 0, np.pi * 3 / 2, np.pi, 0, 0, np.pi, np.pi / 2, np.pi * 3 / 2, np.pi * 3 / 2, np.pi / 2, np.pi, 0, 0, np.pi]
# PACE_VECTOR = [0, 0, 0, 0, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi, -np.pi, 0, 0, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2]
# BOUND_VECTOR = [0, 0, 0, 0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0, np.pi, 0, np.pi, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2]

# 2. frequency
SPEED_STANCE = [1 for i in range(OSCILLATOR_NUMBERS)]
SPEED_SWING = [1 for i in range(OSCILLATOR_NUMBERS)]

# 3. amplitude
SWING_AMPLITUDE = [0, 0, 0.0873 * 2, 0.0873 * 2, 0.025, 0.025, 0.025, 0.025, 0.35 / 2, 0.35 / 2, 0.35 / 2, 0.35 / 2, 0.16, 0.16, 0.16, 0.16]
STANCE_AMPLITUDE = [0, 0, 0.0873 * 2, 0.0873 * 2, 0, 0, 0, 0, 0.35 / 2, 0.35 / 2, 0.35 / 2, 0.35 / 2, 0, 0, 0, 0]

# 4. offset
DESIRED_OFFSET = [0, 0, 0, 0, 0.05, 0.05, 0.05, 0.05, -0.1745, -0.1745, -0.1745, -0.1745, 0, 0, 0, 0]

# Coupling weights, to determine connection structure of the CPG
COUPLING_WEIGHTS = [[0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # reference oscillator has a connection with roll oscillator
                    [CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],  # roll
                    [0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],  # pitch
                    [0, CW_VALUE, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, 0, 0, 0, 0],  # yaw
                    [0, CW_VALUE * 4, CW_VALUE * 4, CW_VALUE * 4, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0, 0],  # RF shoulder
                    [0, CW_VALUE * 4, CW_VALUE * 4, CW_VALUE * 4, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0, 0],  # RR shoulder
                    [0, CW_VALUE * 4, CW_VALUE * 4, CW_VALUE * 4, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, 0],  # LF shoulder
                    [0, CW_VALUE * 4, CW_VALUE * 4, CW_VALUE * 4, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0],  # LR shoulder
                    [0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0, CW_VALUE, 0, 0, 0],  # RF hip
                    [0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0],  # RR hip
                    [0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0],  # LF hip
                    [0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, CW_VALUE, 0, 0, 0, 0, CW_VALUE],  # LR hip
                    [0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, 0, 0, CW_VALUE, CW_VALUE, 0],  # RF knee
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, 0, CW_VALUE, 0, 0, CW_VALUE],  # RR knee
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, 0, 0, CW_VALUE],  # LF knee
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, CW_VALUE, 0, CW_VALUE, CW_VALUE, 0]]  # LR knee

# used for plotting
PLOT_INITIAL_OFFSET = [0 for i in range(OSCILLATOR_NUMBERS)]
PLOT_INITIAL_AMPLITUDE = [0 for i in range(OSCILLATOR_NUMBERS)]
PLOT_INITIAL_PHASE = [0 for i in range(OSCILLATOR_NUMBERS)]
# organising the trot gait parameters
trot_parameters = CpgConfig("trot", OSCILLATOR_NUMBERS, CONSTANT_AR, CONSTANT_AX, TIME_STEP,
                             COUPLING_WEIGHTS, TROT_VECTOR, SPEED_STANCE, SPEED_SWING,
                             STANCE_AMPLITUDE, SWING_AMPLITUDE, DESIRED_OFFSET, INITIAL_AMPLITUDE, INITIAL_OFFSET, INITIAL_PHASE)