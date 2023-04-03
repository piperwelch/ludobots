from robot import ROBOT
import pyrosim.pyrosim as pyrosim
import contextlib
with contextlib.redirect_stdout(None):
    import pybullet as p
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os 
import constants as c
import time
import numpy as np
from scipy.stats import circvar
class SWARM:

    def __init__(self, solutionID, bot_ids):
        self.solutionID = solutionID
        self.robots = {}
        for id in bot_ids:
            self.robots[id] = ROBOT(id)
        self.bot_trajectories = {}

    def compute_heading(self, trajectory):
        # computes headings from x and y coordinates of the trajectory
        # heading is computed between pairs of points and therefore cannot be compute for the first and last points 
        x_pts = [pt[0] for pt in trajectory]
        y_pts = [pt[1] for pt in trajectory]

        headings = []
        for i in range(1,len(x_pts)):
            x_diff = x_pts[i]-x_pts[i-1]
            y_diff = y_pts[i]-y_pts[i-1]
            heading = np.arctan2(y_diff,x_diff)
            headings.append(heading)
        
        return headings

    def compute_straightness_index(self, trajectory):
        # Computes straightness index from headings (circular variance of headings)

        headings = self.compute_heading(trajectory)
        
        # compute circular variance of the headings (between 0-2pi)
        circva = circvar(headings,low=-np.pi, high=np.pi)

        return 1 - circva

    def Get_Fitness(self):
        for id, robot in self.robots.items():
            self.bot_trajectories[id] = robot.trajectory[0]

        trajectories_arr = {}

        for k, v in self.bot_trajectories.items():
            # print(len(v))
            # exit()
            points_touples = v
            points_arr = np.reshape(points_touples, newshape=(len(points_touples),2))
            trajectories_arr[k] = points_arr
        # print(trajectories_arr)
        fitness = self.evaluate_h(trajectories_arr)

        f = open("tmp{}.txt".format(self.solutionID), "w")
        f.write(str(fitness))
        f.close()
        os.system("ren tmp{}.txt fitness{}.txt".format(self.solutionID, self.solutionID))        

    def Get_Fitness_Motile_Bots(self):
        x_min, y_min, x_max, y_max = 1000, 1000, 0, 0

        for pt in self.bot_trajectories[self.solutionID]:
            if pt[0] > x_max:
                x_max = pt[0]
            if pt[0] < x_min:
                x_min = pt[0]
            if pt[1] > y_max:
                y_max = pt[1]
            if pt[1] < y_min:
                y_min = pt[1]

        # print(x_min, y_min, x_max, y_max)
        # exit()
        fitness = self.compute_straightness_index(self.bot_trajectories[self.solutionID])*2 + abs(x_max - x_min) + abs(y_max - y_min)
        f = open("tmp{}.txt".format(self.solutionID), "w")
        f.write(str(fitness))
        f.close()
        os.system("ren tmp{}.txt fitness{}.txt".format(self.solutionID, self.solutionID))        

    def evaluate_h(self,points):

        # Compute the center of the starting points

        # Get initial starting points of all bots
        x_starts = []
        y_starts = []
        # print(type(points))
        count = 0
        for i in points:
            trajectory = points[i]
            x_starts.append(trajectory[0,0])
            y_starts.append(trajectory[0,1])
            # print(i == 0)
            # Also create array of all coordinates for use in computing the bounding box around the trajectories later
            if count == 0:
                all_coords = points[i]
            else:

                all_coords = np.concatenate((all_coords,points[i]))
            count+=1 

        center_x = np.mean(x_starts)
        center_y = np.mean(y_starts)

        # Create a boundary of 20*constants.BOT_LENGTH
        min_x = center_x - c.BOUNDARY_LENGTH_X/2
        max_x = center_x + c.BOUNDARY_LENGTH_X/2
        min_y = center_y - c.BOUNDARY_LENGTH_Y/2
        max_y = center_y + c.BOUNDARY_LENGTH_Y/2

        # Count # unique points
        unique_points = np.unique(all_coords,axis=0)

        # Only keep unique points within the boundary

        # true if in bounds
        x_min_mask = np.reshape(unique_points[:,0]>min_x,newshape=(-1,1))
        x_max_mask = np.reshape(unique_points[:,0]<max_x,newshape=(-1,1))
        y_min_mask = np.reshape(unique_points[:,1]>min_y,newshape=(-1,1))
        y_max_mask = np.reshape(unique_points[:,1]<max_y,newshape=(-1,1))

        mask = np.all(np.concatenate((x_min_mask, x_max_mask, y_min_mask, y_max_mask),axis=1),axis=1)

        unique_points_in_bounds = unique_points[mask]

        return self.fractal_box_count(unique_points_in_bounds,(min_x,min_y,max_x,max_y))

    def fractal_box_count(self, points, boundary):
        # https://francescoturci.net/2016/03/31/box-counting-in-numpy/

        min_x,min_y,max_x,max_y = boundary # unpack tuple

        Ns=[]

        # scales = np.arange(start=2, stop=int(constants.BOUNDARY_LENGTH/constants.MIN_GRID_DIM)) #start with quadrents and go to resolution of voxcraft float
        levels = np.arange(start=1, stop=c.MAX_LEVEL)

        for level in levels: 

            scale = 2**level

            cell_width = c.BOUNDARY_LENGTH_X/scale
            cell_height = c.BOUNDARY_LENGTH_Y/scale

            # H, edges=np.histogramdd(points, bins=(np.linspace(min_x,max_x,constants.BOUNDARY_LENGTH/scale),np.linspace(min_y,max_y,constants.BOUNDARY_LENGTH/scale)))
            H, edges=np.histogramdd(points, bins=(np.linspace(min_x,max_x,num=scale+1),np.linspace(min_y,max_y,num=scale+1)))

            weight = (cell_width*cell_height)/(c.BOUNDARY_LENGTH_X*c.BOUNDARY_LENGTH_Y) # David scaling
            Ns.append(np.sum(H>0)*weight)

        # Divide by # of levels to get a value between 0-1
        scaled_box_count = np.sum(Ns)/len(levels) # David scaling

        return scaled_box_count