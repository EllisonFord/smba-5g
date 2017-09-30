#!/usr/bin/env python3
#
#  _____ _   _ __  __
# |_   _| | | |  \/  |
#   | | | | | | |\/| |
#   | | | |_| | |  | |
#   |_|  \___/|_|  |_|
#
#   ____  __  __ ____    _
#  / ___||  \/  | __ )  / \
#  \___ \| |\/| |  _ \ / _ \
#   ___) | |  | | |_) / ___ \       Created by H. Mirzai, O. El Badramany, M. Riedel on 10th of May 2017
#  |____/|_|  |_|____/_/   \_\
#
#

import numpy as np
import matplotlib.pyplot as plt

# numpy is the library used for mathematical functions

# Global variables
# nR = number of Receivers
# nT = number of Transmitters
# min_SNR and max_SNR are lower and upper bounds for the X-axis in the plot. they will be calculated in another module using the input parameters for rain,temprature,foilage, ...
# Bandwidth frequency in MHz

nR = 20
nT = 20
f_Bandwidth = 80
min_SNR = 0
max_SNR = 30


# initializes the module with the values specified by the user
def init(num_receivers, num_transmitters, frequency_Bandwidth, lowerSNR, upperSNR):
    global nR
    nR = num_receivers
    global nT
    nT = num_transmitters
    global f_Bandwidth
    f_Bandwidth = frequency_Bandwidth
    global min_SNR
    min_SNR = lowerSNR
    global max_SNR
    max_SNR = upperSNR

# TODO: try to use the getChannelMatrix.m script from the NYUSIM tool since it would be much more accurate than just using random rayleigh numbers!
# builds Channel Matrix H.
# I used a rayleigh fading model for implementing the Channel Matrix hence the random rayleigh numbers used.
def generate_Channel_Matrix(nR, nT):
    H = np.zeros((nR, nT), dtype=complex)
    scale = np.sqrt(0.5)
    for x in np.nditer(H, op_flags=['readwrite']):
        x[...] = np.random.rayleigh(scale) + 1j * np.random.rayleigh(scale)
    return H


# this function calculates the implements the Channel Capacity formula for MIMO-Systems.
# it will return the maximum data rate the channel is capable of delivering measured in Gbits/s
# the value of C is multiplied by the frequency bandwidth because what we actually get isn't the max data rate, but the spectral efficiency measured in bits/s/Hz
# ideally we will be able to plot the Channel capacity as a function of SNR , as a function of nT, and as a function of nR

H = generate_Channel_Matrix(nR, nT)


def calculate_Channel_Capacity(avg_SNR):
    I_nR = np.eye(nR, nR, dtype=int)
    A = I_nR + (avg_SNR / nT * (np.dot(H, np.matrix.getH(H))))
    tmp = np.linalg.det(A)
    C = np.real(np.log2(tmp))
    return f_Bandwidth * C


x = []
y = []

for i in range(1, max_SNR + 1):
    c = calculate_Channel_Capacity(i)
    x.append(i)
    y.append(c / 1000)
    if c < 1000:
        print('SNR = ' + str(i) + ' dBm' + '     Channel Capacity = ' + str(c) + ' Mbit/s')
    else:
        print('SNR = ' + str(i) + ' dBm' + '     Channel Capacity = ' + str(c / 1000) + ' Gbit/s')

plt.plot(x,y)
plt.title('Channel Capacity')
plt.xlabel('Signal to Noise Ratio')
plt.ylabel('Channel Capacity in Gbits/s')
plt.show()
