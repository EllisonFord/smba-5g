#!/usr/bin/env python
import matplotlib as mpl
import rospy
from Tkinter import *
from smba_core.msg import *
import matplotlib.pyplot as plt
plt.ion()
from matplotlib.collections import PolyCollection

import matplotlib.patches as patches

# visualizes the currently seen objects and the ground truth
def draw(smba_object_list, fig_num):

    fig = plt.figure(fig_num) # make a new figure with number of sensor
    fig.clear()
    ax = fig.add_subplot(111)


    for i in range(len(smba_object_list)):
        current = smba_object_list[i]
        pos = current.pos
        x = pos.x
        y = pos.y
        height = current.width
        width = current.length
        x_left_bottom = - (width / 2)
        y_left_bottom = - (height / 2)
        rotation_angle = current.heading
        object_type = current.object_type # object type (e.g. car, concrete block)

        if object_type == smba_enums.Car:
            color = [0.02, 0.12, 0.75]
        elif object_type == smba_enums.Tree:
            color = [0.02, 0.67, 0.09]
        elif object_type == smba_enums.Crash_Barrier:
            color = [0.82, 0.01, 0.1]

        rect = patches.Rectangle((x_left_bottom, y_left_bottom), width, height, color=color, alpha=1.0)

        # rotate object by direction in which it is heading
        t = mpl.transforms.Affine2D().rotate(rotation_angle) + ax.transData
        rect.set_transform(t)

        ax.add_patch(rect)
        plt.ylim(-50, 50)
        plt.xlim(-50, 50)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.draw()
    fig.canvas.flush_events()

class listener:
    def __init__(self, sensor_list=[]):

        # this data array is an array of arrays (each sensor has its own array containing his smba_object_list)
        self.data = []

        self.master = Tk()

        rospy.sleep(1.0)
        # we will need a list of IntVars to keep track of the ones selected
        self.chosen_sensors = []

        self.listbox = Listbox(self.master)
        # self.listbox.pack()
        top_list = rospy.get_published_topics()
        for a in top_list:
            if a[1] == 'smba_core/smba_object_list':
                self.listbox.insert(END,a[0])
                sensor_list.append(a[0])

        self.open_GUI(sensor_list)
        rospy.init_node('listener', anonymous=True)
        rate = rospy.Rate(15)  # 1hz

        for i in range(len(sensor_list)):
            # needs to initialize emtpy object in order to write on this space (callback function)
            self.data.append([])
            # subscribe to the sensor in the list
            rospy.Subscriber(sensor_list[i], smba_object_list, self.callback, i)

        while not rospy.is_shutdown():
            self.master.update()
            rate.sleep()
            for i in range(len(sensor_list)):
                if self.chosen_sensors[i].get() == 1:
                    draw(self.data[i], i)
                else:
                    continue

    def update_list(self):
        print "here i am"
        for a in self.chosen_sensors:
            print a.get()

    def open_GUI(self, sensor_list):
        Label(self.master, text="Sensors which are online:").grid(row=0, sticky=W)

        for i in range(len(sensor_list)):
            # get the name of the sensor for labeling checkboxes
            sensor_name = sensor_list[i]
            var = IntVar()
            self.chosen_sensors.append(var)
            var.set(0)
            # append the boolean value to the list to keep track of selected checkboxes
            #self.chosen_sensors.append(BooleanVar())
            # create a checkbox with the sensor name in the next row
            Checkbutton(self.master, text=sensor_name, variable=var).grid(row=i+1, sticky=W)
            del var

        Button(self.master, text='Apply', command=self.update_list).grid(row=((len(sensor_list)) + 2), sticky=W, pady=4)

    def callback(self, data, i):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.head.msg_id)
        #val = data.head.last_modification - data.head.generation_time
        #rospy.loginfo("Last Sensor Simulation took %f seconds", data.head.last_modification - data.head.generation_time)
        self.data[i] = data.data # write the data of the sensor into the space of the data array

if __name__ == '__main__':
    #sensor_list = ['ground_truth', 'sensor_1'] # list containing only ground truth and the default sensor now
    listener()
