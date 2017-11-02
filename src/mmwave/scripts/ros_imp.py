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

import rospy
from visualization_msgs.msg import MarkerArray
#from smba_core.msg import smba_object_list


default_topic    = "/ground_truth"
default_msg_type = "visualization_msgs/MarkerArray"

# keeps track of how many vehicles exist so that all of the table does not have to be re-drawn for every new message. Default 1.
vehicle_number = 1

vehicle_data = []


# Sets the table default values, formats the size of the columns and the number of rows that will appear.
def set_ros_table():

    global vehicle_number

    print("Drawing the table for " + str(vehicle_number) + " vehicle/s.")

    #create here a list that has as many items as the number of vehicles
    del vehicle_data[:]

    for i in range(0, vehicle_number):

        if i % 2 == 0:
            colour = colour_even
        else:
            colour = colour_odd

        vehicle_data.append(
            [tkinter.StringVar(), tkinter.StringVar(), tkinter.StringVar(), tkinter.StringVar(), tkinter.StringVar()])

        tkinter.Label(master, text="ID",                        bg=colour, height=1, width=2,  anchor='w').grid(row=i+1, column=4)
        tkinter.Label(master, textvariable=vehicle_data[i][0],  bg=colour, height=1, width=3,  anchor='w').grid(row=i+1, column=5)

        tkinter.Label(master, text="Distance to Tx:",           bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=6)
        tkinter.Label(master, textvariable=vehicle_data[i][1],  bg=colour, height=1, width=9,  anchor='w').grid(row=i+1, column=7)

        tkinter.Label(master, text="Vehicle Data:",             bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=8)  # how much data the vehicle wants to offload to the grid
        tkinter.Label(master, textvariable=vehicle_data[i][2],  bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=9)

        tkinter.Label(master, text="Transfer Speed:",           bg=colour, height=1, width=12, anchor='w').grid(row=i+1, column=10) # how much can the tower provide
        tkinter.Label(master, textvariable=vehicle_data[i][3],  bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=11)

        tkinter.Label(master, text="Vehicle type:",             bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=12)
        tkinter.Label(master, textvariable=vehicle_data[i][4],  bg=colour, height=1, width=10, anchor='w').grid(row=i+1, column=13)



def callback(data):

    # Checks if ROS is running and if the published topic can be heard, it also changes the icon in the GUI
    check_topic_status()

    global vehicle_number

    # if the number of vehicles has changed then run the set_ros_table() function again
    if len(data.markers) != vehicle_number:
        vehicle_number = len(data.markers)
        set_ros_table()


    if default_msg_type == "visualization_msgs/MarkerArray":
        pass # take the variables into a list
    elif default_msg_type == "smba/smba_object_list":
        pass # take the variables into a list
    else:
        pass

    from model_functions import *
    from mimo import *



    for i, vehicle in enumerate(data.markers):

        # Calculations
        distance_calculation = np.hypot(data.markers[i].pose.position.x, data.markers[i].pose.position.y)

        print("distance_calc:",distance_calculation, "carrier_freq:", carrier_freq, "path_loss_exp:",path_loss_exp)

        path_loss_result = path_loss(tr_distance=distance_calculation, carrier_freq=carrier_freq, pl_exponent=path_loss_exp)

        print("path_loss: ", path_loss_result)

        friss_result = friis(losses=(path_loss_result + rain_loss(rain.get()) + foilage_loss(carrier_freq.get(), foilage.get())), tx_power=power_db.get(), tx_gain=trans_gain.get(), rx_gain=receiv_gain.get())

        print("friss: ", friss_result)

        #snr_result = snr(signal=friss_result, noise=(nyquist_noise(freq_band.get(), temperature.get())))

        #c = calculate_Channel_Capacity(snr_result, no_transmitters.get(), no_receivers.get(), freq_band.get()) / 1000



        #print(snr_result, no_transmitters.get(), no_receivers.get(), freq_band.get())

        # Setting the labels
        vehicle_data[i][0].set(data.markers[i].id)
        vehicle_data[i][1].set(round(distance_calculation, decimal_places))
        vehicle_data[i][2].set(round(1.234, decimal_places))
        #vehicle_data[i][3].set(round(c, decimal_places))
        vehicle_data[i][4].set(data.markers[i].text)




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
            status_message.set(32*" ") #TODO: Improve this to a remove statement
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