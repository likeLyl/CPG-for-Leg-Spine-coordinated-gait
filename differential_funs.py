# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : differential_funs.py
# Date      : 27-Nov-2023
# Version:  : 
# ----------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np


class DifferentialEq:
    def __init__(self, ar, R, stp_size, pos=0) -> None:
        self.dy_1 = 0
        self.dy_2 = 0
        self.R = R
        self.ar = ar

        self.pos = pos
        self.stepsize = stp_size

    def update_amp_1(self):  # CPG driven locomotion control of quadruped robot
        self.dy_2 = 4 * (self.R - self.pos) - 3 / 2 * 2 * self.dy_1
        self.dy_1 += self.dy_2 * self.stepsize
        self.pos += self.dy_1 * self.stepsize
        return self.pos

    def update_amp_2(self):  # controlling swimming and crawling in a fish robot...
        a = 20
        self.dy_2 = self.ar * (self.ar / 4 * (self.R - self.pos) - self.dy_1)
        self.dy_1 += self.dy_2 * self.stepsize
        self.pos += self.dy_1 * self.stepsize
        return self.pos

    def update_amp_3(self):  # towards dynamic trot gait locomotion: design, control, and experiments with Cheetah-cub...
        self.dy_1 = self.ar * (self.R - self.pos)
        self.pos += self.dy_1 * self.stepsize
        return self.pos

    def update_amp_4(self):  # learning robot gait stability using neural networks as sensory feedback function...
        self.dy_1 = self.ar * (self.R - self.pos * self.pos) * self.pos
        self.pos += self.dy_1 * self.stepsize
        return self.pos

    def update_amp_5(self):
        self.dy_1 = self.ar * (self.R - self.pos) ** 3
        self.pos += self.dy_1 * self.stepsize
        return self.pos

    @staticmethod
    def plot_r(y):
        l = len(y)
        # print(t_end)
        t = np.arange(0, l, 1)
        # print(l,':', t_end, ":", t_interval)
        plt.figure(figsize=(16, 8))
        # right front leg
        plt.plot(t, y, color='red')
        # plt.title(title[0])
        # plt.legend(loc="right")
        # plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    amp = DifferentialEq(10, 2, 1 / 60)
    x = []
    dy_2 = []
    dy_1 = []
    for i in range(800):
        if i == 500:
            amp.pos = 1
        x.append(amp.update_amp_3())
        dy_1.append(amp.dy_1)
        dy_2.append(amp.dy_2)

    # amp.plot_r(dy_2)
    amp.plot_r(dy_1)
    amp.plot_r(x)