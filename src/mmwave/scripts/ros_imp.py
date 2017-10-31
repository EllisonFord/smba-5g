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
import numpy as np
try:
    import tkinter
except:
    import Tkinter as tkinter # python2

from model_functions import *

import rospy
from visualization_msgs.msg import MarkerArray
#from smba_core.msg import smba_object_list


default_topic    = "/ground_truth"
default_msg_type = "visualization_msgs/MarkerArray"

# keeps track of how many vehicles exist so that all of the table does not have to be re-drawn for every new message. Default 1.
vehicle_number = 1

id     = tkinter.IntVar()
dist_m = tkinter.DoubleVar()
bandw  = tkinter.DoubleVar()
v_type = tkinter.StringVar()
v_data = tkinter.DoubleVar()

vehicle_data = [[id, dist_m, bandw, v_type, v_data], [id, dist_m, bandw, v_type, v_data], []]


# Sets the table default values, formats the size of the columns and the number of rows that will appear.
def set_ros_table():

    global vehicle_number

    print("Drawing the table for " + str(vehicle_number) + " vehicle/s.")

    id.set(0)
    dist_m.set(0.0)
    v_data.set(0.0)
    bandw.set(0.0)
    v_type.set("No data.")

    #TODO: Each label has to have its own variable, or it won't work when you need to update them.

    #create here a list that has as many items as the number of vehicles

    for i in range(0, vehicle_number):

        if i % 2 == 0:
            colour = colour_even
        else:
            colour = colour_odd

        tkinter.Label(master, text="ID",                 bg=colour, height=1, width=2,  anchor='w').grid(row=i+1, column=4)
        tkinter.Label(master, textvariable=id,           bg=colour, height=1, width=3,  anchor='w').grid(row=i+1, column=5)

        tkinter.Label(master, text="Distance to Tx:",    bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=6)
        tkinter.Label(master, textvariable=dist_m,       bg=colour, height=1, width=9,  anchor='w').grid(row=i+1, column=7)

        tkinter.Label(master, text="Vehicle Data:",      bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=8) # how much data the vehicle wants to offload to the grid
        tkinter.Label(master, textvariable=v_data,       bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=9)

        tkinter.Label(master, text="Transfer Speed:",    bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=10) # how much can the tower provide
        tkinter.Label(master, textvariable=bandw,        bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=11)

        tkinter.Label(master, text="Vehicle type:",      bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=12)
        tkinter.Label(master, textvariable=v_type,       bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=13)




def callback(data):

    # Checks if ROS is running and if the published topic can be heard, it also changes the icon in the GUI
    check_topic_status()

    global vehicle_number

    # if the number of vehicles has changed then run the set_ros_table() function again
    if len(data.markers) != vehicle_number:
        vehicle_number = len(data.markers)
        set_ros_table()

    for i, vehicle in enumerate(data.markers):

        # Calculations
        distance_calculation = np.hypot(data.markers[i].pose.position.x, data.markers[i].pose.position.y)

        path_loss_result = path_loss(distance_m.get(), carrier_freq.get(), path_loss_exp.get())

        friss_result = friis(path_loss_result + rain_loss(rain.get()) + foilage_loss(carrier_freq.get(), foilage.get()), power_db.get(), trans_gain.get(), receiv_gain.get())

        snr_result = snr(friss_result, nyquist_noise(freq_band.get(), temperature.get()))

        c = calculate_Channel_Capacity(avg_SNR=snr_result, nT=no_transmitters.get(), nR=no_receivers.get(), f_Bandwidth=freq_band.get()) / 1000

        # Setting the labels
        id.set(data.markers[i].id)

        dist_m.set(round(distance_calculation, decimal_places))

        v_data.set(round(1.234, decimal_places))

        bandw.set(round(c, decimal_places))

        v_type.set(data.markers[i].text)




def listener():

    rospy.init_node('mmWave_node', anonymous=True)

    rospy.Subscriber(default_topic, MarkerArray, callback)




def check_topic_status():

    global default_topic
    global default_msg_type
    global status_icon

    #print("Checking ROS status.")
    status_message = tkinter.StringVar()

    topic_list = rospy.get_published_topics()

    topic_names         = [name for (name, type) in topic_list]

    topic_message_types = [type for (name, type) in topic_list]

    # Place the initial labels
    tkinter.Label(master, image=status_icon,   border=0          ).grid(row=1, column=2)
    tkinter.Label(master, textvariable=status_message, bg='white').grid(row=2, column=2)



    if default_topic in topic_names:
        if default_msg_type not in topic_message_types:
            status_icon = status_red
            status_message.set("Unexpected message type")
            return False
        else:
            status_icon = status_green
            status_message.set("                        ") #TODO: Improve this to a remove statement
            return True
    else:
        if topic_list == None:
            status_icon = status_yellow
            status_message.set("roscore not running")
            return False

        elif default_topic not in topic_list:
            status_icon = status_orange
            status_message.set("Default ROS topic not found")
            return False

        else: # if smba_core.msg.smba_object_list not in topics
            status_icon = status_red
            status_message.set("ROS error")
            return False
