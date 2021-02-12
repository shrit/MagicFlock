from numpy import genfromtxt
import math
import textwrap
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.manifold import TSNE
from io import StringIO
import numpy as np
import pandas as pd
import sys

def plot_dataset_tsne(dataset_file_name, dataset_file_name2, dataset_file_name3, episdes_file_name, episdes_file_name2, episdes_file_name3):

    train_matrix = genfromtxt(dataset_file_name, delimiter=',')
    train_2_matrix = genfromtxt(dataset_file_name2, delimiter=',')
    train_3_matrix = genfromtxt(dataset_file_name3, delimiter=',')

    Big_matrix = np.concatenate((train_matrix, train_2_matrix, train_3_matrix))
    print("Original shape: 1", train_matrix.shape)
    print("Original shape: 2", train_2_matrix.shape)
    print("Original shape: 3", train_3_matrix.shape)
    print("Shape of big matrix", Big_matrix.shape)

    data_1_color_vec = np.loadtxt(episdes_file_name)
    data_2_color_vec = np.loadtxt(episdes_file_name2)
    data_3_color_vec = np.loadtxt(episdes_file_name3)

    tsne = TSNE(n_components=2, random_state=0)
    mat = tsne.fit_transform(Big_matrix)
    print(mat.shape)

    data_1  = mat[:np.size(train_matrix,0), :]
    print("Original shape: ", train_matrix.shape, "New Shape: ", data_1.shape)
    print ("Mat shape", mat.shape) 
    data_2 = mat[np.size(train_matrix, 0):np.size(train_matrix, 0) + np.size(train_2_matrix, 0) , :]
    print("Original shape: ", train_2_matrix.shape, "New Shape: ", data_2.shape)

    data_3 = mat[np.size(train_2_matrix,0): np.size(train_2_matrix, 0) + np.size(train_3_matrix, 0), :]
    print("Original shape: ", train_3_matrix.shape, "New Shape: ", data_3.shape)

    plt.figure()
    plt.scatter(data_1[:,0], data_1[:, 1], c=data_1_color_vec, cmap='plasma')
    plt.colorbar()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(23, 18)
    plt.savefig(dataset_file_name + ".svg", dpi=600, format="svg")

    plt.figure()
    plt.scatter(data_2[:,0], data_2[:, 1], c=data_2_color_vec, cmap='plasma')
    plt.colorbar()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(23, 18)
    plt.savefig(dataset_file_name2 + ".svg", dpi=600, format="svg")

    plt.figure()
    plt.scatter(data_3[:,0], data_3[:, 1], c=data_3_color_vec, cmap='plasma')
    plt.colorbar()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(23, 18)
    plt.savefig(dataset_file_name3 + ".svg", dpi=600, format="svg")

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

    y = np.loadtxt(generic_file_name)
    x = np.arange(y.size)

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
    
def plot_3d_generic(generic_file_name):
    num_lines = sum(1 for line in open(generic_file_name))
    print (num_lines)

    dataset = np.loadtxt(generic_file_name, delimiter=",")
    x = dataset[:, 0]
    y = dataset[:, 1]
    z = dataset[:, 2]
    return x,y,z

def plot_3D_position(*file_names):
    figure = plt.figure()
    ax = figure.gca(projection='3d') # get current figure
    ax.legend()
    # ax.set_xlim3d(-100, 100)
    # ax.set_ylim3d(-100,100)
    ax.set_zlim3d(0,100)
    for file_name in file_names:
      a, b, c = plot_3d_generic(file_name)
      ax.plot(a, b, c)

    figure.set_size_inches(12, 8)
    plt.savefig(file_name + ".svg", dpi=100)

def plot_2D_position(*file_names):
    figure = plt.figure()
    ax = figure.gca(projection='3d') # get current figure
    ax.legend()
    # ax.set_xlim3d(-100, 100)
    # ax.set_ylim3d(-100,100)
    ax.set_zlim3d(0,100)
    for file_name in file_names:
      a, b = plot_2d_generic(file_name)
      ax.plot(a, b)

    figure.set_size_inches(12, 8)
    plt.savefig(file_name + ".svg", dpi=100)

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

def plot_cumulative_histogram(*histogram_file_names):

    for histogram_file_name in histogram_file_names:
      x, y = cumulative_histogram(histogram_file_name)
      plt.plot(x, y)

    plt.title("Cumulative distribution function")
    plt.xlabel('Number of time steps executed by the follower per episode')
    plt.ylabel('Percentage of episodes')
    plt.legend()
    plt.grid()
    figure = plt.gcf() # get current figure
    figure.set_size_inches(12, 8)
    plt.savefig(histogram_file_name + "cumulative_.svg", dpi=100)

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

    parser.add_argument('--dataset_file_name', metavar="dataset file name", type=str, nargs="+", help="Enter dataset file name to plot")
    parser.add_argument('--position_files_name', metavar="3D position file name", type=str, nargs="+", help="Enter 3d position files to plot")
    parser.add_argument('--tsne_dataset_file_name', metavar="dataset file name", type=str, nargs="+", help="Enter dataset file name to plot using tsne")
    parser.add_argument('--generic_file_name', metavar="generic file name", type=str, help="Enter a generic file to plot with one column")
    parser.add_argument('--error_file_name', metavar="error file name", type=str, help="Enter error file name that has the mean value of each flight")
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

    elif args.dataset_file_name:
        modify_labels(args.dataset_file_name)

    elif args.position_files_name:
        plot_3D_position(*args.position_files_name)

    elif args.tsne_dataset_file_name:
        plot_dataset_tsne(args.tsne_dataset_file_name[0], args.tsne_dataset_file_name[1], args.tsne_dataset_file_name[2], args.tsne_dataset_file_name[3], args.tsne_dataset_file_name[4], args.tsne_dataset_file_name[5])

    elif args.histogram_file_name:
        plot_histogram_2d(args.histogram_file_name)

    elif args.cumulative_histogram_files_name:
        plot_cumulative_histogram(*args.cumulative_histogram_files_name)
