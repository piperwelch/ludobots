import numpy as np 
import matplotlib.pyplot as plt
import pickle
import pandas as pd
import os
import pickle
from scipy.stats import circvar


def compute_heading(x,y):
    # computes headings from x and y coordinates of the trajectory
    # heading is computed between pairs of points and therefore cannot be compute for the first and last points 
    x_pts = x
    y_pts = y

    headings = []
    for i in range(1,len(x_pts)):
        x_diff = x_pts[i]-x_pts[i-1]
        y_diff = y_pts[i]-y_pts[i-1]
        heading = np.arctan2(y_diff,x_diff)
        headings.append(heading)
    
    return headings

def compute_straightness_index(x,y):
    # Computes straightness index from headings (circular variance of headings)

    headings = compute_heading(x,y)
    
    # compute circular variance of the headings (between 0-2pi)
    civ = circvar(headings,low=-np.pi, high=np.pi)

    return 1 - civ
for i in range(1):
    with open("bot_pool_trajectories/pickle{}.p".format(i), 'rb') as f:
        x, y = pickle.load(f)

    plt.plot(x,y)
plt.show()
plt.xlabel("X distance traveled")
plt.ylabel("Y distance traveled")
plt.title("Pool of 500 bots")
