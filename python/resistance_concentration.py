import numpy as np
import matplotlib.pyplot as plt




# Parameters
C_min = 1.0
C_max = 300.0
Rs_R0_min = 0.03
Rs_R0_max = 2.0
RL = 76000 # lowest 820, 76kOhm or 51.1kOhm
R0_min = 100000
R0_max = 1500000
b = -0.632
a = 0.109
Vs = 3

C = np.linspace(C_min, C_max, 500)

Rs_R0 = (10**a) * C**b
assert np.all(Rs_R0 >= Rs_R0_min) and np.all(Rs_R0 <= Rs_R0_max), \
    f"Rs_R0: {Rs_R0} not in [{Rs_R0_min}, {Rs_R0_max}]"

fig, axes = plt.subplots(ncols=3, nrows=3, figsize=(30, 20))
for i, R0 in enumerate(np.linspace(R0_min, R0_max, axes.shape[1])):
    Vm = Vs / ((R0/RL) * 10**a * C**b + 1)
    Vm *= 1000

    ax = axes[0,i]
    ax.plot(Vm, C)
    ax.set_ylabel('C [ppm]')
    ax.set_xlabel('V [mV]')
    ax.set_title(f'R0: {np.round(R0/1000)}k Ohm')

    ax = axes[1,i]
    ax.plot(Vm, C)
    ax.set_ylabel('C [ppm]')
    ax.set_xlabel('V [mV]')
    ax.set_yscale('log')

    ax = axes[2,i]
    ax.plot(Vm, C)
    ax.set_ylabel('C [ppm]')
    ax.set_xlabel('V [mV]')
    ax.set_yscale('log')
    ax.set_xscale('log')

plt.show()




# # Parameters
# a = 0.109
# b = -0.632 #-0.368
# RL = 900 #51000 # lowest 820
# R0 = 100000 # typical 100000-1500000
# Vs = 5

# Vm = np.linspace(0.01, Vs, 100)

# C = ((RL/((10**a)*R0)) * ((Vs/Vm) - 1))**(1/b)

# Rs = RL*((Vs/Vm) - 1)
# print(f"b: {b}")
# log10_C = (np.log10(Rs/R0) - a) / b
# assert np.allclose(C, 10**log10_C)

# # Plot
# fig, axes = plt.subplots(ncols=2, nrows=1, figsize=(10, 5))

# ax = axes[0]
# ax.plot(Vm, C)
# ax.set_xlabel('Vm')
# ax.set_ylabel('C')
# ax.set_title('C vs Vm')

# # plot log of C
# ax = axes[1]
# ax.plot(Vm, C)
# ax.set_xlabel('Vm')
# ax.set_ylabel('C')
# ax.set_title('C vs Vm')
# # ax.set_yscale('log')
# # ax.set_xscale('log')
# ax.set_xlim(0.0, 1.0)
# ax.set_ylim(0.0, 300.0)

# plt.show()
