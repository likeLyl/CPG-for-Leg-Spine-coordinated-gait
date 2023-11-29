# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : cpg_configuration.py
# Date      : 27-Nov-2023
# Version:  : 
# ----------------------------------------------------------------
import numpy as np


class CpgConfig:
    def __init__(self, NAME: str, OSCILLATOR_NUMBERS: int, CONSTANT_AR: int, CONSTANT_AX: int, TIME_STEP: float,
                 COUPLING_WEIGHTS: list, PHASE_VECTOR: list, SPEED_STANCE: list, SPEED_SWING: list,
                 STANCE_AMPLITUDE: list, SWING_AMPLITUDE: list, DESIRED_OFFSET: list,
                 INITIAL_AMPLITUDE: list, INITIAL_OFFSET: list, INITIAL_PHASE: list) -> None:
        self.name = NAME
        self.oscillator_number = OSCILLATOR_NUMBERS

        # So that the phase lag can be edited
        self.phase_lag_vector = PHASE_VECTOR

        # matrix
        self.coupling_weights = COUPLING_WEIGHTS
        self.phase_lag_matrix = self.create_phase_matrix(PHASE_VECTOR)
        self.speed_stance = SPEED_STANCE
        self.speed_swing = SPEED_SWING
        self.st_amplitude = STANCE_AMPLITUDE
        self.sw_amplitude = SWING_AMPLITUDE
        self.desired_offset = DESIRED_OFFSET
        self.initial_amplitude = INITIAL_AMPLITUDE
        self.initial_offset = INITIAL_OFFSET
        self.initial_phase = INITIAL_PHASE

        # define constant variables
        self.ar = CONSTANT_AR
        self.ax = CONSTANT_AX
        self.time_step = TIME_STEP

    def create_phase_matrix(self, initial_vector):
        assert self.oscillator_number == len(initial_vector)
        matrix = np.zeros((self.oscillator_number, self.oscillator_number))
        matrix[0] = initial_vector
        # print(len(matrix[0]))
        for i in range(1, len(matrix)):
            for j in range(i + 1, len(matrix[i])):
                matrix[i][j] = initial_vector[j] - initial_vector[i]
        for i in range(1, len(matrix)):
            for j in range(0, i):  # i=12, j=0
                matrix[i][j] = -matrix[j][i]
        return matrix