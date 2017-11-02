#!/usr/bin/env python
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
import os.path as path
from PIL import Image, ImageTk
try:
    import tkinter
except:
    import Tkinter as tkinter # python2

master = tkinter.Tk()
master.title("5G Network Simulator for Autonomous Vehicles")
master.geometry("1630x600") # size of the window in px, the width is divided by 1.618 to calculate the height
colour = 'white'
master.configure(background = colour) # background colour
button_size = 18

decimal_places = 2


CARR_FREQ   = 0
FREQ_BAND   = 1
NT          = 2
NR          = 3
DIST        = 4
TPOWER      = 5
TGAIN       = 6
RGAIN       = 7
FOILAGE     = 8
TEMP        = 9
RAIN        = 10
WEATHER     = 11
PATHLOSSEXP = 12

labels   = [
        "Carrier Frequency [GHz]",
        "Bandwidth [MHz]",
        "Number of Transmitters",
        "Number of Receivers",
        "TX-RX Distance [m]",
        "TX Power [dB]",
        "TX Gain [dBi]",
        "RX Gain [dBi]",
        "Foilage depth [m] (0...400)",
        "Temperature [C]",
        "Rainfall [mm/hr]",
        "Weather attenuation factor (1 . . 0)",
        "Path Loss Exp (1 . . 4)",
        "Environment"
        ]

defaults = [
        28.0,
        200.0,
        1,
        1,
        500.0,
        30.0,
        10.0,
        10.0,
        0.0,
        20.0,
        0.0,
        0.0,
        2.0,
        None
        ]



environment = [["Urban LOS", "Urban NLOS", "Rural LOS", "Rural NLOS"],  # Type of environment
               [2.0,            3.2,            2.16,       2.75],      # Path loss exponentials?
               [4.0,            7.0,            4.0,        8.0]]       #


colour_even = "light sky blue"
colour_odd  = "white"

# Makes a relative
dir = path.dirname(__file__)
white_logo  = path.join(dir, 'imgs/logo_white.gif')
mini_green  = path.join(dir, 'imgs/mini_g.gif')
mini_yellow = path.join(dir, 'imgs/mini_y.gif')
mini_orange = path.join(dir, 'imgs/mini_o.gif')
mini_red    = path.join(dir, 'imgs/mini_r.gif')

# makes global variables for the check_topic_status() function to use
status_green  = ImageTk.PhotoImage(Image.open(mini_green))
status_yellow = ImageTk.PhotoImage(Image.open(mini_yellow))
status_orange = ImageTk.PhotoImage(Image.open(mini_orange))
status_red    = ImageTk.PhotoImage(Image.open(mini_red))
status_icon = status_yellow
