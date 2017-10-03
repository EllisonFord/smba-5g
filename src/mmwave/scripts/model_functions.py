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

#import numpy as np
from mimo import *

# Free Space Path Loss (1m reference)
def free_space_path_loss_1m(carrier_freq):
    return 32.4 + 20 * np.log10(carrier_freq) # def. 28 GHz


# Log Normal Shadowing Path Loss
#n: Path loss exponent (1 < n < 5)
#chi: standard variation fadin (0 < chi < 20)
def lognormal_path_loss(carrier_freq, tr_distance, pl_exponent, chi = 4):
    return free_space_path_loss_1m(carrier_freq) + 10 * pl_exponent * np.log10(tr_distance) + chi


# TODO:Weather attenuation factor
def weather_att(att = 0):
    return att


# Foilage Attenuation:
# (Weissberger's model)
#The MED model is found to be applicable to cases in which the ray path is
#blocked by dense, dry, in-leaf trees found in temperate-latitude forests

#d: # foilage depth (max 400m)
def foilage_loss(carrier_freq, foilage_depth):
    if 14 < foilage_depth <= 400:
        return 1.33 * (carrier_freq ** 0.284) * (foilage_depth ** 0.588)
    elif 0 <= foilage_depth <= 14:
        return 0.45 * (carrier_freq ** 0.284) * foilage_depth
    else:
        # Assume max 400m foilage
        print("Input distance not in range 0 to 400. lel")
        return 1.33 * (carrier_freq ** 0.284) * (400 ** 0.588)

# Rainfall attenuation
# rain: rainfall in mm/hr
def rain_loss(rain):
    theta = 0 # 0 deg elevation
    tau = 0 # horizontal polarization
    kh = 0.2051 # 28Ghz coefficients
    ah = 0.9679
    kv = 0.1964
    av = 0.9277
    k = (kh + kv + (kh - kv) * np.cos(theta)**2 * np.cos(2 * tau) ) / 2
    a = (kh * ah + kv * av + ( kh * ah - kv * av) * np.cos(theta) **2 * np.cos(2 * tau) ) / 2 * k
    return (k * rain ** a) / 1000 #return in db/m


# Total Path loss:
def path_loss(tr_distance, carrier_freq, pl_exponent):
    return lognormal_path_loss(carrier_freq, tr_distance, pl_exponent, chi = 0) # + AT + fl(carrier_freq, foilage_depth = 10)


def friis(losses, tx_power, tx_gain, rx_gain):
    res = tx_power + tx_gain + rx_gain - losses
    return res

def nyquist_noise(bandwidth, temp = 20):
    bw = bandwidth * 1e6 # Bandwidth given in MHz
    kb = 1.38065e-23 #Boltzmann Constant
    temp_kelvin = 273.15 + temp
    return 10 * np.log10(kb * temp_kelvin * 1e3) + 10 * np.log10(bw)

def snr_db(signal, noise):
    res = signal + noise
    print(signal, noise, res)
    return res

def snr(signal, noise):
    # convert dbm to power (watts)
    s = 10**((signal-30)/10)
    n = 10**((noise-30)/10)
    return s/n

def shannon_capacity(bandwidth, snr):
    bw = bandwidth * 1e6 # Bandwidth given in MHz
    return bw * np.log2(1 + snr)



def mimo_print_src(snr, nt, nr, freq_bandwidth):
    c = calculate_Channel_Capacity(snr, nt, nr, freq_bandwidth)  # Answer is in Mbit/s
    if c < 1000.:
        print('SNR = ' + str(snr) + ' dBm' + '     Channel Capacity = ' + str(c) + ' Mbit/s')
    else:
        print('SNR = ' + str(snr) + ' dBm' + '     Channel Capacity = ' + str(c / 1000) + ' Gbit/s')
    return c




distance = 500
carrier_frequency = 28.0
pathloss_exp      = 1
rain_density      = 50
foilage_density   = 100
transmiter_power  = 30
transmiter_gain   = 10
receiver_gain     = 10
temperature       = 20
bandwidth         = 200
num_transmiters   = 5
num_receivers     = 5


mimo_print_src(snr(friis(path_loss(500, 28.0, 1) + rain_loss(50) + foilage_loss(28.0, 100), 30, 10, 10), nyquist_noise(200, 20)), 5, 5, 200)


print(calculate_Channel_Capacity(snr(friis(path_loss(500, 28.0, 1) + rain_loss(50) + foilage_loss(28.0, 100), 30, 10, 10), nyquist_noise(200, 20)), 5, 5, 200))
print(calculate_Channel_Capacity(snr(friis(path_loss(500, 28.0, 1) + rain_loss(50) + foilage_loss(28.0, 100), 30, 10, 10), nyquist_noise(200, 20)), 5, 5, 200))
print(calculate_Channel_Capacity(snr(friis(path_loss(500, 28.0, 1) + rain_loss(50) + foilage_loss(28.0, 100), 30, 10, 10), nyquist_noise(200, 20)), 5, 5, 200))

