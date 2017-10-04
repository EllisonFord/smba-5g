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

import rospy
from visualization_msgs.msg import MarkerArray
#from smba_core.msg import smba_object_list


default_topic    = "/ground_truth"
default_msg_type = "visualization_msgs/MarkerArray"

# keeps track of how many vehicles exist so that all of the table does not have to be re-drawn for every new message. Default 1.
vehicle_number = 1


# Sets the table default values, formats the size of the columns and the number of rows that will appear.
def set_ros_table():

    global vehicle_number

    for i in range(0, vehicle_number):

        if i % 2 == 0:
            colour = colour_even
        else:
            colour = colour_odd

        Label(master, text="ID",                        bg=colour, height=1, width=2,  anchor='w').grid(row=i+1, column=4)
        Label(master, text="{}".format(0),              bg=colour, height=1, width=3,  anchor='w').grid(row=i+1, column=5)

        Label(master, text="Distance to Tx:",           bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=6)
        Label(master, text="{} m".format(0),            bg=colour, height=1, width=9,  anchor='w').grid(row=i+1, column=7)

        Label(master, text="Vehicle Data:",             bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=8) # how much data the vehicle wants to offload to the grid
        Label(master, text="{} Gbit/s".format(0),       bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=9)

        Label(master, text="Transfer Speed:",           bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=10) # how much can the tower provide
        Label(master, text="{} Gbit/s".format(0),       bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=11)

        #Label(master, text="Dropped:",                  bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=12) # how much can the tower provide
        #Label(master, text="{}%".format(0),             bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=13)

        Label(master, text="Vehicle type:",             bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=12)
        Label(master, text="{}".format("No data."),     bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=13)



def callback(data):

    # Checks if ROS is running and if the published topic can be heard, it also changes the icon in the GUI
    check_topic_status()

    global vehicle_number

    # if the number of vehicles has changed then run the set_ros_table() function again
    if len(data.markers) != vehicle_number:
        vehicle_number = len(data.markers)
        set_ros_table()

    for i, vehicle in enumerate(data.markers):

        if i % 2 == 0:
            colour = colour_even
        else:
            colour = colour_odd

        distance_m = np.hypot(data.markers[i].pose.position.x, data.markers[i].pose.position.y)

        c = calculate_Channel_Capacity(snr(friis(path_loss(distance_m, carrier_freq, 2) + rain_loss(rain) + foilage_loss(carrier_freq, foilage), power_db, trans_gain, receiv_gain), nyquist_noise(freq_band, temperature)), no_transmitters, no_receivers, freq_band)

        Label(master, text="{}".format(data.markers[i].id),                   bg=colour, height=1, width=3,  anchor='w').grid(row=i+1, column=5)

        Label(master, text="{} m".format(round(distance_m, decimal_places)),  bg=colour, height=1, width=9,  anchor='w').grid(row=i+1, column=7)

        Label(master, text="{} Gbit/s".format(round(1.234, decimal_places)),  bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=9)

        Label(master, text="{} Gbit/s".format(round(c/1000, decimal_places)), bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=11)

        #Label(master, text="{}%".format(round(0.123, decimal_places)),        bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=13)

        Label(master, text="{}".format(data.markers[i].text),                 bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=13)




def listener():

    rospy.init_node('mmWaveNode', anonymous = True)

    rospy.Subscriber(default_topic, MarkerArray, callback)




def check_topic_status():

    global default_topic
    global default_msg_type

    global status_green
    global status_yellow
    global status_orange
    global status_red


    topic_list = rospy.get_published_topics()

    topic_names         = [name for (name, type) in topic_list]

    topic_message_types = [type for (name, type) in topic_list]

    Label(master, text="                                          ", bg=colour     ).grid(row=2, column=2)

    if default_topic in topic_names:

        if default_msg_type not in topic_message_types:
            Label(master, image=status_red, border=0                               ).grid(row=1, column=2)
            Label(master, text="Unexpected message type", bg=colour                ).grid(row=2, column=2)
            return False

        else:
            Label(master, image=status_green, border=0                             ).grid(row=1, column=2)
            return True

    else:
        if topic_list == None:

            Label(master, image=status_yellow, border=0                            ).grid(row=1, column=2)
            Label(master, text="roscore not running", bg=colour                    ).grid(row=2, column=2)
            return False

        elif default_topic not in topic_list:

             Label(master, image=status_orange, border=0                           ).grid(row=1, column=2)
             Label(master, text="Default ROS topic not found", bg=colour, width= 25).grid(row=2, column=2)
             return False

        else: # if smba_core.msg.smba_object_list not in topics

            Label(master, image=status_red, border=0                               ).grid(row=1, column=2)
            Label(master, text="ROS error", bg=colour                              ).grid(row=2, column=2)
            return False

