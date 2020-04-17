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

def plot_total_error(time, error):
    plt.figure()
    plt.plot(time, error, color="red")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Error (metres)") #THE YAW IS ALSO TAKEN INTO ACCOUNT -> THINK ABOUT ONLY USING EUCLIDEAN DISTANCE 
    #SMOOTH THE ZIG ZAG CURVE - FIND A WAY AND DISPLAY BOTH 

def plot_parametric_error(time, x1, x2, y1, y2, yaw1, yaw2):
    #find error values for x, y and yaw for each time step 
    N = len(time)
    errorX = []
    errorY = []
    errorYaw = []

    #error for every timestep 
    for i in range(0, N):
        errorX.append(abs(x2[i]-x1[i]))
        errorY.append(abs(y2[i]-y1[i]))
        errorYaw.append(abs(yaw2[i]-yaw1[i])) #NEED TO TAKE INTO ACCOUNT ANGLES LIKE -2.50 and 2.50 -> very close, NOT very far
    errorX = np.asarray(errorX)
    errorY = np.asarray(errorY)
    errorYaw = np.asarray(errorYaw)

    #plot x and y
    plt.figure()
    plt.plot(time, errorX, label="X Error", color="red")
    plt.plot(time, errorY, label="Y Error", color="blue")
    plt.ylabel("Error (metres)")
    plt.xlabel("Time (seconds)")
    plt.legend()

    #plot yaw
    plt.figure()
    plt.plot(time, errorYaw, label="Yaw Error", color="green")
    plt.ylabel("Error (radians)")
    plt.xlabel("Time (seconds)")
    plt.legend()

def plot_two_paths(x1, y1, x2, y2, label1, label2="Ground Truth"):
    plt.figure()
    plt.plot(x1, y1, label=label1, color="red")
    plt.plot(x2, y2, label=label2, color="blue")
    plt.xlabel("X (metres)")
    plt.ylabel("Y (metres)")
    plt.legend()

def plot_three_paths(x1, y1, x2, y2, x3, y3, label1="Kinematics Estimation", label2="EKF Estimation", label3="Ground Truth"):
    plt.figure()
    plt.plot(x1, y1, label=label1, color="red")
    plt.plot(x2, y2, label=label2, color="blue")
    plt.plot(x3, y3, label=label3, color="green")
    plt.xlabel("X (metres)")
    plt.ylabel("Y (metres)")
    plt.legend()

def plot_cone_map(x1,y1,x2,y2):
    plt.figure()
    plt.plot(x1, y1, 'rx', label="FastSLAM2.0")
    plt.plot(x2, y2, 'bo', mfc='none', label="Ground Truth")
    plt.xlabel("X (metres)")
    plt.ylabel("Y (metres)")
    plt.legend()

def plot_landmarks(x,y):
    plt.figure()
    plt.plot(x, y, 'rx', label="FastSLAM2.0")
    plt.xlabel("X (metres)")
    plt.ylabel("Y (metres)")
    plt.legend()


def save_plot(output_path):
    global data
    global column_names

    plt.style.use('seaborn-darkgrid')

    #STYLES: https://matplotlib.org/3.1.1/gallery/style_sheets/style_sheets_reference.html
    if len(column_names) == 8:
        #plot_paths(np.asarray(data[0]), np.asarray(data[1]), np.asarray(data[3]), np.asarray(data[4]), "Kinematics Estimation")
        #plot_total_error(np.asarray(data[6]), np.asarray(data[7]))
        #plot_parametric_error(np.asarray(data[6]), np.asarray(data[0]), np.asarray(data[3]), np.asarray(data[1]), np.asarray(data[4]), 
        #    np.asarray(data[2]), np.asarray(data[5]))
        plot_three_paths(np.asarray(data[0]), np.asarray(data[1]), np.asarray(data[2]), np.asarray(data[3]),
            np.asarray(data[4]), np.asarray(data[5]))
        plot_total_error(np.asarray(data[6]), np.asarray(data[7]))


    elif len(column_names) == 6: #ground truth coordinates, prediction coordinates, error over time
        plt.figure()
        plt.plot(np.asarray(data[0]), np.asarray(data[1]), label="EKF Estimation", color="red")
        plt.plot(np.asarray(data[2]), np.asarray(data[3]), label="Ground Truth", color="blue")
        plt.xlabel("X (metres)")
        plt.ylabel("Y (metres)")
        plt.legend()

        plt.figure()
        plt.plot(np.asarray(data[4]), np.asarray(data[5]), color="red")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Error (metres)")
        #plt.title("Differential-Drive Kinematics vs Ground Truth Position Estimation")

    elif len(column_names) == 4: #ground truth coordinates, prediction coordinates
        plot_cone_map(np.asarray(data[0]), np.asarray(data[1]), np.asarray(data[2]), np.asarray(data[3]))
        #plt.title("Differential-Drive Kinematics vs Ground Truth Position Estimation")

    elif len(column_names) == 2: #error over time
        plot_landmarks(np.asarray(data[0]), np.asarray(data[1]))           
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

