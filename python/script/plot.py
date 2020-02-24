from numpy import genfromtxt
import math
import textwrap
import argparse
import matplotlib.pyplot as plt
from sklearn.manifold import TSNE
from io import StringIO
import numpy as np
import pandas as pd
import sys

def plot_dataset_tsne(dataset_file_name):
    train_matrix = genfromtxt(StringIO(dataset_file_name), delimiter=',')
    tsne = TSNE(n_components=2, random_state=0)

    mat = tsne.fit_transform(train_matrix)

    print(mat.shape)
    plt.scatter(mat[:,0], mat[:,1], color='blue')
    figure = plt.gcf() # get current figure
    figure.set_size_inches(25, 12)

    plt.savefig(dataset_file_name + ".png", dpi=100, format="png")
    
def drop_columns(dataset_file_name, column_nubmer):
    df = pd.read_csv(dataset_file_name)

    df.drop([df.columns[0],
             df.columns[9]],
            axis = 1, inplace = True)

    df.to_csv(dataset_file_name_dropped, index = False)


def modify_labels(dataset_file_name):
    dataset = genfromtxt(dataset_file_name, delimiter=',')
    y = dataset[:, -1]
    dataset = dataset[:, :-1]
    print(y)
    z =[]
    for i in y:
        z.append(math.log10(i + 1e-7))

    new_data_set = np.column_stack((dataset, z))
    np.savetxt(dataset_file_name, new_data_set, delimiter=",")


def plot_generic(generic_file_name, xlabel, ylabel):
    num_lines = sum(1 for line in open(generic_file_name))
    print (num_lines)

    y = np.loadtxt(generic_file_name)
    x = np.arange(y.size)

    print(y)
    z = np.mean(y)

    print ("standard deviation : ", np.std(y))
    print("mean value:", z)

    plt.plot(x, y, color='blue')

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    figure = plt.gcf() # get current figure
    figure.set_size_inches(10, 6)

    plt.savefig(generic_file_name + ".png", dpi=100, format="png")

def plot_generic(generic_file_name):
    num_lines = sum(1 for line in open(generic_file_name))
    print (num_lines)

    y = np.loadtxt(generic_file_name)
    x = np.arange(y.size)
    return x,y

def plot_loss(loss_file_name):
    mat = genfromtxt(loss_file_name, delimiter=',')
    real_loss = mat[:,9]
    predicted_loss = mat[:, 4]
    d1_p_e = mat[:, 0]
    d2_p_e = mat[:, 1]
    d3_p_e = mat[:, 2]
    h_p_e  = mat[:, 3]

    d1_r_e = mat[:, 5]
    d2_r_e = mat[:, 6]
    d3_r_e = mat[:, 7]
    h_r_e  = mat[:, 8]

    x = np.arange(len(real_loss));

    #plt.plot(x, real_loss, color='blue', label="real loss")
    #plt.plot(x, predicted_loss, color='red', label="predicted loss")
    plt.plot(x, d1_p_e, color='navy', label="prediction error on d1")
    plt.plot(x, d2_p_e, color='green', label="prediction error on d2")
    plt.plot(x, d3_p_e, color='purple', label="prediction error on d3")
    plt.plot(x, h_p_e, color='yellow', label="prediction error on delta h")

    plt.plot(x, d1_r_e, color='darkred', label="real error on d1")
    plt.plot(x, d2_r_e, color='brown', label="real error on d2")
    plt.plot(x, d3_r_e, color='orange',   label="real error on d3")
    plt.plot(x, h_r_e, color='black',  label="real error on delta h")

    plt.title("Real error vs prediction error")
    plt.xlabel('Time steps')
    plt.ylabel('Error in meter')
    plt.legend()
    plt.grid()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(21, 18)
    plt.savefig(loss_file_name + ".svg", dpi=100)    
    
def plot_six_generic(file_name,
                     file_name_2,
                     file_name_3,
                     file_name_4,
                     file_name_5,
                     file_name_6):
    a, b = plot_generic(file_name)
    c, d = plot_generic(file_name_2)
    e, f = plot_generic(file_name_3)
    g, h = plot_generic(file_name_4)
    i, j = plot_generic(file_name_5)
    k, l = plot_generic(file_name_6)
    plt.plot(a, b, color='blue', label="forward")
    plt.plot(c, d, color='orange', label="backward")
    plt.plot(e, f, color='green', label="left")
    plt.plot(g, h, color='black', label="right")
    plt.plot(i, j, color='red', label="up")
    plt.plot(k, l, color='brown', label="down")

    plt.title("Comparing distance between alice and charlie on different actions")
    plt.xlabel('sample number: 1 second equal 20')
    plt.ylabel('Distance between alice and charlie')
    plt.legend()
    plt.grid()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(12, 8)
    plt.savefig(file_name + ".png", dpi=100)

"""
Error files need to be formated as one column file
This column contain the mean error of each flight
"""
def plot_flight_error(error_file_name):
    num_lines = sum(1 for line in open(error_file_name))
    print (num_lines)

    y = np.loadtxt(error_file_name)
    x = np.arange(y.size)

    print(y)
    z = np.mean(y)

    print ("error standard deviation : ", np.std(y))
    print(z)

    plt.plot(x, y, color='blue')

    plt.xlabel('Number of flight')
    plt.ylabel('Error of deformation combined totatly in m')

    figure = plt.gcf() # get current figure
    figure.set_size_inches(25, 6)

    plt.savefig(error_file_name + ".svg", dpi=100, format="svg")

"""
Count files need to be formated as one column file
This column contain the number of steps of each flight
"""

def plot_flight_count(count_file_name):

    num_lines = sum(1 for line in open(count_file_name))
    print (num_lines)

    y = np.loadtxt(count_file_name)
    x = np.arange(y.size)
    z = np.mean(y)

    print ("count deviation: ", np.std(y) )
    print(z)

    plt.plot(x, y, color='blue')

    plt.xlabel('Number of flights')
    plt.ylabel('Controller Counts before deformation')

    figure = plt.gcf() # get current figure
    figure.set_size_inches(25, 6)

    plt.savefig(count_file_name + ".png", dpi=100)


def plot_histogram(count_file_name, histogram_file_name):
    x = np.loadtxt(count_file_name)
    plt.hist(x, bins=3)

    plt.xlabel('Number of time steps per episode')
    plt.ylabel('Frequency')

    figure = plt.gcf()
    figure.set_size_inches(25, 6)

    plt.savefig(histogram_file_name + ".png", dpi=100)


def plot_histogram_2d(histogram_file_name):

    data = pd.read_csv(histogram_file_name, sep=' ',header=None, index_col =0)

    data.plot(kind='bar')
    plt.xlabel('Number of time steps per episode')
    plt.ylabel('Frequency')

    figure = plt.gcf() # get current figure
    figure.set_size_inches(10, 6)

    plt.savefig(histogram_file_name + ".png", dpi=100)

def cumulative_histogram(filename):
    data = genfromtxt(filename, delimiter=' ')
    x = data[:, 0:1]
    y = data[:, 1:2]
    frequency_sum = np.sum(y)
    z = []
    for i in range(1, len(y)+1):
        z.append(np.sum(y[0:i]) / frequency_sum)

    return x, z

def plot_one_cumulative_histogram(histogram_file_name):
    x, z = cumulative_histogram(histogram_file_name)

    plt.plot(x, z, color='blue')
    plt.xlabel('Number of time steps per episode')
    plt.ylabel('CDF')
    plt.grid()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(25, 6)
    plt.savefig(histogram_file_name + "one_cumulative_.png", dpi=100)

def plot_two_cumulative_histogram(histogram_file_name,
                                  histogram_file_name_2):
    x, y = cumulative_histogram(histogram_file_name)
    i, j = cumulative_histogram(histogram_file_name_2)

    plt.plot(x, y, color='blue', label="Phase 1")
    plt.plot(i, j, color='green', label="Phase 2")

    plt.title("Cumulative distribution function of random and trained controller")
    plt.xlabel('Number of time steps executed by the follower per episode')
    plt.ylabel('cdf')
    plt.legend()
    plt.grid()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(12, 8)
    plt.savefig(histogram_file_name + "two_cumulative_.svg", dpi=100)

def plot_three_cumulative_histogram(histogram_file_name,
                                    histogram_file_name_2,
                                    histogram_file_name_3):
    x, y = cumulative_histogram(histogram_file_name)
    a, b = cumulative_histogram(histogram_file_name_2)
    i, j = cumulative_histogram(histogram_file_name_3)
    plt.plot(x, y, color='blue', label="Phase 1")
    plt.plot(a, b, color='orange', label="Phase 2")
    plt.plot(i, j, color='green', label="Oracle")

    plt.title("Cumulative distribution function of random, knn and trained controller")
    plt.xlabel('Number of time steps executed by the follower per episode')
    plt.ylabel('Percentage of episodes')
    plt.legend()
    plt.grid()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(12, 8)
    plt.savefig(histogram_file_name + "three_cumulative_.svg", dpi=100)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        prog='Plotting script',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent('''\
        PLotting script!
        --------------------------------
        This script is used to plot the result generated by the
        quadcopters in the simulator
        It can be used to plot one of the files,
        or all of them at the same time.
        You have to provide the file format as described in the above comments.
        '''))

    parser.add_argument('--dataset_file_name', metavar="dataset file name", type=str, help="Enter dataset file name to plot")
    parser.add_argument('--tsne_dataset_file_name', metavar="dataset file name", type=str, help="Enter dataset file name to plot using tsne")
    parser.add_argument('--loss_file_name', metavar="loss file name", type=str, help="Enter loss file name to plot")
    parser.add_argument('--generic_file_name', metavar="generic file name", type=str, help="Enter a generic file to plot with one column")
    parser.add_argument('--six_generic_file_name', metavar="generic file name", type=str, nargs="+", help="Enter a generic file to plot with one column")
    parser.add_argument('--error_file_name', metavar="error file name", type=str, help="Enter error file name that has the mean value of each flight")
    parser.add_argument('--count_file_name', metavar="count file name", type=str, help="Enter flight count file name  ")
    parser.add_argument('--histogram_file_name', metavar="histogram file name", type=str, help="Enter a histogram file name, two column file name, nuumber of steps and frequency")
    parser.add_argument('--cumulative_histogram_files_name', metavar=" histogram file name", type=str, nargs="+" ,help="Enter one or more histogram file name, each file is a two column file name, number of steps and frequency, the script will generate automatically the cumulative histogram")
    parser.add_argument('--drop_columns', metavar=" drop column", type=int ,help="Enter column number to delete from dataset")
    args = parser.parse_args()

    plt.rcParams.update({'font.size': 9})

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    if args.error_file_name:
        plot_flight_error(args.error_file_name)

    elif args.generic_file_name:
        xlabel = input('Enter your xlabel:')
        ylabel = input('Enter your ylabel:')
        plot_generic(args.generic_file_name, xlabel, ylabel)

    elif args.six_generic_file_name:
        plot_six_generic(args.six_generic_file_name[0],
                         args.six_generic_file_name[1],
                         args.six_generic_file_name[2],
                         args.six_generic_file_name[3],
                         args.six_generic_file_name[4],
                         args.six_generic_file_name[5])

    elif args.count_file_name:
        plot_flight_count(args.count_file_name)

    elif args.dataset_file_name:
        modify_labels(args.dataset_file_name)

    elif args.tsne_dataset_file_name:
        plot_dataset_tsne(args.dataset_file_name)
        
    elif args.loss_file_name:
        plot_loss(args.loss_file_name)

    elif args.histogram_file_name:
        plot_histogram_2d(args.histogram_file_name)

    elif args.cumulative_histogram_files_name:
        if isinstance(args.cumulative_histogram_files_name, str):
            plot_one_cumulative_histogram(args.cumulative_histogram_files_name)
        elif isinstance(args.cumulative_histogram_files_name, list):
            plot_two_cumulative_histogram(args.cumulative_histogram_files_name[0],
                                          args.cumulative_histogram_files_name[1])
                                         # args.cumulative_histogram_files_name[2])
