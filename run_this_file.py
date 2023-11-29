# ----------------------------------------------------------------
# The University of york
# The School of Physics, Engineering and Technology
# Robotics and Autonomous System Lab
# Author    : Yunlong Lian, PhD students
# File      : run_this_file.py
# Date      : 27-Nov-2023
# Version:  : 
# ----------------------------------------------------------------
from trot_parameters import trot_parameters
from basic_cpg import BasicCpg

# initialize CPG for test
test_cpg = BasicCpg(trot_parameters)
theta_s = [[] for i in range(test_cpg.numbers)]
r_s = [[] for i in range(test_cpg.numbers)]
phi_s = [[] for i in range(test_cpg.numbers)]

for k in range(800):
    # update CPG
    test_cpg.update_x()
    test_cpg.update_r()
    phi_pos = test_cpg.update_phi()
    theta_v = test_cpg.update_setpoints()
    # recording parameters
    for i in range(test_cpg.numbers):
        r_s[i].append(test_cpg.amp_out[i])
        phi_s[i].append(test_cpg.phi[i])

    for i in range(len(theta_v)):
        theta_s[i].append(theta_v[i])
# plot shoulder, hip and knee joints results
label = [['θ 4: Right Front', 'θ 5: Right Rear', 'θ 6: Left Front', 'θ 7: Left Rear'],
         ['θ 8: Right Front', 'θ 9: Right Rear', 'θ 10: Left Front', 'θ 11: Left Rear'],
         ['θ 12: Right Front', 'θ 13: Right Rear', 'θ 14: Left Front', 'θ 15: Left Rear']]
title = ['Shoulder Joint', 'Hip Joint', 'Knee Joint']
test_cpg.plot_all_results(theta_s[1:4], theta_s[4:8], theta_s[8:12], theta_s[12:16], 411, label, title)

leg_label = [['Shoulder1', 'Hip5', 'Knee9'],
         ['Shoulder2', 'Hip6', 'Knee10'],
         ['Shoulder3', 'Hip7', 'Knee11'],
         ['Shoulder4', 'Hip8', 'Knee12']]
title = ['right front', 'right rear', 'left front', 'left rear']
test_cpg.plot_each_leg(theta_s[4:13:4], theta_s[5:14:4], theta_s[6:15:4], theta_s[7:16:4], 411, leg_label, title)
print("Done")