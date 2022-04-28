#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.backend_bases import NavigationToolbar2

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

colors = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
]

COL_TYPE = 0
COL_ID = 1
COL_X = 2
COL_Y = 3
COL_Z = 4
COL_T = 5

TYPE_ELEVATOR = 0
TYPE_BIN = 1
TYPE_ROBOT = 2
TYPE_PATH = 3


def prev_plot(self, *args, **kwargs):
    print("prev_plot")


i = 0


def next_plot(self, *args, **kwargs):
    print("next_plot")
    global i
    ax.plot([i], [i], "s")
    plt.draw()
    i += 1


NavigationToolbar2.back = prev_plot
NavigationToolbar2.forward = next_plot

parser = argparse.ArgumentParser(description="plot data")
parser.add_argument("data")
args = parser.parse_args()

# parse csv
data = np.genfromtxt(args.data, delimiter=",", skip_header=1)
# extract elevator entries
elevators = data[data[:, COL_TYPE] == TYPE_ELEVATOR]
# extract floors (sorted array of unique z values)
floors = np.unique(data[:, COL_Z])
# set axis ticks to max range of x and y
x_ticks = np.arange(data[:, COL_X].max() + 1)
y_ticks = np.arange(data[:, COL_Y].max() + 1)
t_ticks = np.arange(data[:, COL_T].max() + 1)
aspect = (np.ptp(data[:, COL_X]), np.ptp(data[:, COL_Y]), np.ptp(data[:, COL_T]))

# each floor has it's own plot figure
for floor in floors:
    # extract only entries corresponding the current floor
    floor_data = data[data[:, COL_Z] == floor]
    # extract location of bins on current floor
    bins = floor_data[floor_data[:, COL_TYPE] == TYPE_BIN]
    # extract location of bots on current floor
    bots = floor_data[floor_data[:, COL_TYPE] == TYPE_ROBOT]
    # extract location of bots on current floor
    paths = floor_data[floor_data[:, COL_TYPE] == TYPE_PATH]

    # setup figure
    fig = plt.figure(num=f"Floor {int(floor)}")
    ax = fig.add_subplot(projection="3d")
    ax.set_zlim3d(0)
    ax.set_box_aspect(aspect)
    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)
    ax.zaxis.set_ticks(t_ticks)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    '''
    # plot markers for elevators robots and bins
    ax.plot(
        elevators[:, COL_X], elevators[:, COL_Y], "^", color="purple", markersize=10
    )
    ax.plot(bins[:, COL_X], bins[:, COL_Y], "s", color="tan", markersize=10)
    ax.plot(bots[:, COL_X], bots[:, COL_Y], "o", color="red", markersize=10)

    # label markers with their ID
    for i, label in enumerate(elevators[:, COL_ID]):
        ax.text(elevators[i, COL_X], elevators[i, COL_Y], 0, int(label))
    for i, label in enumerate(bins[:, COL_ID]):
        ax.text(bins[i, COL_X], bins[i, COL_Y], 0, int(label), ha="center", va="center")
    for i, label in enumerate(bots[:, COL_ID]):
        ax.text(bots[i, COL_X], bots[i, COL_Y], 0, int(label), ha="center", va="center")
        '''

    # extract unique paths on current floor
    for path_id in np.unique(paths[:, COL_ID]):
        path = paths[paths[:, COL_ID] == path_id]
        if len(path) == 1:
            continue
        ax.quiver(
            path[:-1, COL_X],
            path[:-1, COL_Y],
            path[:-1, COL_T],
            np.diff(path[:, COL_X]),
            np.diff(path[:, COL_Y]),
            np.diff(path[:, COL_T]),
            color=colors[int(path_id) % len(colors)],
        )

plt.grid()
plt.show()
