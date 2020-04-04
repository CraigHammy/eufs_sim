#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv 
import rospy

def load_csv(path):
    global data
    global column_names
    with open(path) as file:
        reader = csv.DictReader(file, delimiter=',')
        #csv file should only have 2 columns one for x and one for y axis
        column_names = next(reader)
        num_axes = len(column_names)
        for row in reader:
            element = []
            for i in range(num_axes):
                element.append(row[column_names[i]])
            data.append(element)
    np.asarray(data)

def save_plot(output_path):
    global data
    global column_names
    #STYLES: https://matplotlib.org/3.1.1/gallery/style_sheets/style_sheets_reference.html
    plt.figure(figsize=(20, 20))
    if len(column_names) == 4:
        plt.plot(data[0], data[1], label="kinematics estimation", color="red")
        plt.plot(data[2], data[3], label="ground truth estimation", color="blue")
        plt.xlabel("X (metres)")
        plt.ylabel("Y (metres)")
        plt.legend()
        #plt.title("Differential-Drive Kinematics vs Ground Truth Position Estimation")
    elif len(column_names) == 2:
        plt.plot(data[0], data[1], color="red")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Error")            
        #plt.title("Error")
    plt.imsave(output_path)


if __name__ == "__main__":
    global data
    global column_names 
    data = []

    rospy.init_node("visualising_data_node")
    output_path = rospy.get_param("~output_path")
    input_path = rospy.get_param("~input_path")
    load_csv(input_path)
    save_plot(output_path)

