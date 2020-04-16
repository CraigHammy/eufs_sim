#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv 
import rospy
import pandas as pd

def load_csv(path):
    global data
    global column_names

    df = pd.read_csv(path)
    column_names = df.columns
    for column in column_names:
        data.append(df[column])
  
def save_plot(output_path):
    global data
    global column_names

    plt.style.use('seaborn-darkgrid')

    #STYLES: https://matplotlib.org/3.1.1/gallery/style_sheets/style_sheets_reference.html
    if len(column_names) == 6: #ground truth coordinates, prediction coordinates, error over time
        plt.figure()
        plt.plot(np.asarray(data[0]), np.asarray(data[1]), label="Kinematics Estimation", color="red")
        plt.plot(np.asarray(data[2]), np.asarray(data[3]), label="Ground Truth", color="blue")
        plt.xlabel("X (metres)")
        plt.ylabel("Y (metres)")
        plt.legend()

        plt.figure()
        plt.plot(np.asarray(data[4]), np.asarray(data[5]), color="red")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Error (metres)")
        #plt.title("Differential-Drive Kinematics vs Ground Truth Position Estimation")

    if len(column_names) == 4: #ground truth coordinates, prediction coordinates
        plt.figure(figsize=(20, 20))
        plt.plot(data[0], data[1], label="Kinematics Estimation", color="red")
        plt.plot(data[2], data[3], label="Ground Truth", color="blue")
        plt.xlabel("X (metres)")
        plt.ylabel("Y (metres)")
        plt.legend()
        #plt.title("Differential-Drive Kinematics vs Ground Truth Position Estimation")
    elif len(column_names) == 2: #error over time
        plt.figure(figsize=(20, 20))
        plt.plot(data[0], data[1], color="red")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Error")            
        #plt.title("Error")
    plt.show()
    #plt.savefig(output_path)
    #print("saved")


if __name__ == "__main__":
    global data
    global column_names 
    data = []

    rospy.init_node("visualising_data_node")
    output_path = rospy.get_param("~output_path")
    input_path = rospy.get_param("~input_path")
    load_csv(input_path)
    save_plot(output_path)

