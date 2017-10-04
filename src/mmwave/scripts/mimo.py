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
#   ___) | |  | | |_) / ___ \       Created by H. Mirzai, O. El Badramany on 10th of May 2017
#  |____/|_|  |_|____/_/   \_\
#
#


from model import *


# Global variables
# nR = number of Receivers
# nT = number of Transmitters
# min_SNR and max_SNR are lower and upper bounds for the X-axis in the plot. they will be calculated in another module using the input parameters for rain,temprature,foilage, ...
# Bandwidth frequency in MHz


# this function calculates the implements the Channel Capacity formula for MIMO-Systems.
# it will return the maximum data rate the channel is capable of delivering measured in Gbits/s
# the value of C is multiplied by the frequency bandwidth because what we actually get isn't the max data rate, but the spectral efficiency measured in bits/s/Hz
# ideally we will be able to plot the Channel capacity as a function of SNR , as a function of nT, and as a function of nR
# TODO: try to use the getChannelMatrix.m script from the NYUSIM tool since it would be much more accurate than just using random rayleigh numbers!
# builds Channel Matrix H.
# I used a rayleigh fading model for implementing the Channel Matrix hence the random rayleigh numbers used.
def calculate_Channel_Capacity(avg_SNR, nR, nT, f_Bandwidth):
    H = np.zeros((nR, nT), dtype=complex)
    scale = np.sqrt(0.5)
    for x in np.nditer(H, op_flags=['readwrite']):
        x[...] = np.random.rayleigh(scale) + 1j * np.random.rayleigh(scale)
    I_nR = np.eye(nR, nR, dtype=int)
    A = I_nR + (avg_SNR / nT * (np.dot(H, np.matrix.getH(H))))
    tmp = np.linalg.det(A)
    C = np.real(np.log2(tmp))
    return f_Bandwidth * C



def plot_mimo(min_SNR, max_SNR, bandwidth = 200, num_transmiters = 20, num_receivers = 20):

    x = y = []

    for i in range(min_SNR, max_SNR + 1):

        c = calculate_Channel_Capacity(i, num_receivers, num_transmiters, bandwidth)
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
