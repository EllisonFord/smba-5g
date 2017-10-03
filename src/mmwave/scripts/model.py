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
# Order of the imports affects behaviour

from tkinter import *
from model_functions import *
from project_constants import *
#from mimo import *
import matplotlib.pyplot as plt
import numpy as np
from ros_imp import *



#TODO: Fix the crashing/freezing of the application by making nested frames in the main window
#TODO: Add a return to defaults button
#TODO: Add sliders to the GUI: Will help since all the input parameters have max and min limits
#TODO: fix the delay in the tkinter Labels: Try compiling the code see if that improves it
#TODO: Make a ROS launchfile for everything
#TODO: Comment the code


# Global variables created and set to default values
carrier_freq    = defaults[CARR_FREQ]
freq_band       = defaults[FREQ_BAND]
no_transmitters = defaults[NT]
no_receivers    = defaults[NR]
distance_m      = defaults[DIST]
power_db        = defaults[TPOWER]
trans_gain      = defaults[TGAIN]
receiv_gain     = defaults[RGAIN]
foilage         = defaults[FOILAGE]
temperature     = defaults[TEMP]
rain            = defaults[RAIN]
weather_att     = defaults[WEATHER]
environ_param1  = environment[1][0]
environ_param2  = environment[2][0]


entry_list      = [] # the GUI form entries


def set_defaults():
    global carrier_freq
    carrier_freq = defaults[CARR_FREQ]
    global freq_band
    freq_band = defaults[FREQ_BAND]
    global no_transmitters
    no_transmitters = defaults[NT]
    global no_receivers
    no_receivers = defaults[NR]
    global distance_m
    distance_m = defaults[DIST]
    global power_db
    power_db = defaults[TPOWER]
    global foilage
    foilage = defaults[FOILAGE]
    global temperature
    temperature = defaults[TEMP]
    global rain
    rain = defaults[RAIN]
    global weather_att
    weather_att = defaults[WEATHER]
    global environ_param1
    environ_param1 = environment[1][0]
    global environ_param2
    environ_param2 = environment[2][0]


def userinput():

    check_topic_status() # Checks if ROS is running and if the published topic can be heard, it also changes the icon in the GUI

    for i, entry in enumerate(entry_list):
        if i == CARR_FREQ:
            global carrier_freq
            carrier_freq    = float(entry.get())
        if i == FREQ_BAND:
            global freq_band
            freq_band       = float(entry.get())
        if i == NT:
            global no_transmitters
            no_transmitters = int(entry.get())
        if i == NR:
            global no_receivers
            no_receivers    = int(entry.get())
        if i == DIST:
            global distance_m
            distance_m      = float(entry.get())
        if i == TPOWER:
            global power_db
            power_db        = float(entry.get())
        if i == TGAIN:
            global trans_gain
            trans_gain      = float(entry.get())
        if i == RGAIN:
            global receiv_gain
            receiv_gain     = float(entry.get())
        if i == FOILAGE:
            global foilage
            foilage         = float(entry.get())
        if i == TEMP:
            global temperature
            temperature     = float(entry.get())
        if i == RAIN:
            global rain
            rain            = float(entry.get())
        if i == WEATHER:
            global weather_att
            weather_att     = float(entry.get())




def option_func(value):

    global environment
    global environ_param1
    global environ_param2

    if value == "Urban LOS":
        environ_param1 = environment[1][0] # assigns 2.0
        environ_param2 = environment[2][0] # assigns 4.0
    elif value == "Urban NLOS":
        environ_param1 = environment[1][1] # returns 3.2
        environ_param2 = environment[2][1] # returns 7.0
    elif value == "Rural LOS":
        environ_param1 = environment[1][2] # returns 2.16
        environ_param2 = environment[2][2] # returns 4.0
    else:
        environ_param1 = environment[1][3]  # returns 2.75
        environ_param2 = environment[2][3]  # returns 8.0


def plot_func():

    userinput()

    d1 = np.arange(10, distance_m, 10)  # start 0, end 400, step 10

    plt.figure(1)
    plt.subplot(211)
    plt.ylabel('Path Loss [dB]')
    plt.xlabel('distance [m]')

    ### Path Loss Plots ###

    pl_exp = 1

    # Log Normal w/ Path Loss
    dat1 = []
    for d in d1:
        dat1.append(path_loss(d, carrier_freq, pl_exp))

    plt.plot(d1, dat1, 'r.', label='Free Space Path Loss')

    # Log Normal + Foilage Path Loss
    dat2 = []
    for d in d1:
        dat2.append(path_loss(d, carrier_freq, pl_exp) + foilage_loss(carrier_freq, foilage))

    plt.plot(d1, dat2, 'g.', label='PL + Foilage')

    # Log Normal + Rain Loss
    dat3 = []
    for d in d1:
        dat3.append(path_loss(d, carrier_freq, pl_exp) + rain_loss(rain))

    plt.plot(d1, dat3, 'y.', label='PL + Rain')

    plt.legend(loc='upper right')

    ### SNR Plots ###

    # plt.subplot(212)
    # plt.ylabel('snr');

    # dat10 = []
    # for d in d1:
    #    dat10.append(
    #            snr_db(
    #                friis(path_loss(d, carrier_freq, pl_exp), tx_power_db, 10, 10),
    #                nyquist_noise(bandwidth)
    #                )
    #            )
    # plt.plot(d1, dat10, 'r.', label='LOS');

    # dat11 = []
    # for d in d1:
    #    dat11.append(
    #            snr_db(
    #                friis(path_loss(d, carrier_freq, pl_exp), tx_power_db, 10, 10) + foilage_loss(carrier_freq, d_foilage),
    #                nyquist_noise(bandwidth)
    #                )
    #            )
    # plt.plot(d1, dat11, 'g.', label='LOS');

    ### Channel Capacity Plots ###

    plt.subplot(212)
    plt.ylabel('Channel Capacity [b/s]')
    plt.xlabel('distance [m]')

    # Path Loss
    dat4 = []
    for d in d1:
        dat4.append(
            shannon_capacity(freq_band,
                             snr(
                                 friis(path_loss(d, carrier_freq, pl_exp), power_db, trans_gain, receiv_gain),
                                 nyquist_noise(freq_band, temperature)
                             )
                             )
        )

    plt.plot(d1, dat4, 'r.', label='Free Space Path Loss')

    # Path Loss + Foilage
    dat5 = []
    for d in d1:
        dat5.append(
            shannon_capacity(freq_band,
                             snr(
                                 friis(path_loss(d, carrier_freq, pl_exp) + foilage_loss(carrier_freq, foilage),
                                       power_db, trans_gain, receiv_gain),
                                 nyquist_noise(freq_band, temperature)
                             )
                             )
        )

    plt.plot(d1, dat5, 'g.', label='PL + Foilage')

    # Path Loss + Rain
    dat6 = []
    for d in d1:
        dat6.append(
            shannon_capacity(freq_band,
                             snr(
                                 friis(path_loss(d, carrier_freq, pl_exp) + rain_loss(rain), power_db, trans_gain,
                                       receiv_gain),
                                 nyquist_noise(freq_band, temperature)
                             )
                             )
        )

    plt.plot(d1, dat6, 'y.', label='PL + Rain')

    plt.legend(loc='upper right')

    plt.show()




logo       = PhotoImage(file=white_logo)
logo_panel = Label(master, image=logo, border=0).grid(row = 0, column = 2)



# Fill in Col = 0 with labels and Col = 1 with Entry fields
for i, label in enumerate(labels):
    Label(master, text=label, bg=colour, height = 1, width = 27).grid(row=i+1, column=0) # Place labels on column 0
    if i == len(labels) - 1:
       a = OptionMenu(master, StringVar(), *environment[0], command=option_func)
       a.config(width = 16)
       a.grid(row = i+1, column = 1)
    else:
        # e. stands for Entry
        e = Entry(master, textvariable=StringVar(), width=20)
        e.grid(row = i+1, column = 1)
        e.insert(0, defaults[i])    # index, default values
        entry_list.append(e) # add the entry to the entry_list array so that it may be used in other parts of the app




button_ros_display = Button(master, text="Submit to ROS",        command = userinput,   height = 0,  width = button_size).grid(row = len(labels)+1, column = 1, padx=0, pady=10)
button_plot        = Button(master, text="Tower-Receiver Plot",  command = plot_func,   height = 0,  width = button_size).grid(row = len(labels)+2, column = 1, padx=0, pady=0)
button_quit        = Button(master, text="Quit",                 command = master.quit, height = 1,  width = 15         ).grid(row = len(labels)+3, column = 2, padx=0, pady=100)



if __name__ == '__main__':

    # Prints the ROS table on the right side of the GUI
    set_ros_table()

    # Checks if ROS is running and if the published topic can be heard, it also changes the icon in the GUI
    check_topic_status()

    # Begins the subscriber
    listener()

    # Runs the GUI and ROS on a loop created by tkinter
    mainloop()



