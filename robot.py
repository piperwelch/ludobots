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
class ROBOT:

    def __init__(self, solutionID):
        self.solutionID = solutionID
        self.robotId = ""
        self.trajectory = {0:[]}
        while self.robotId == "":
            try:
                self.robotId = p.loadURDF("bot_pool/body{}.urdf".format(solutionID))
            except:
                time.sleep(0.01)


        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        self.nn = NEURAL_NETWORK("bot_pool/brain{}.nndf".format(self.solutionID))
        
        # os.system("del brain{}.nndf".format(self.solutionID))
        # os.system("del body{}.urdf".format(self.solutionID))


    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for k,v in self.sensors.items():
            v.Get_Value(t)

    def Get_position(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)    
        self.trajectory[0].append((basePositionAndOrientation[0][0], basePositionAndOrientation[0][1]))


    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[str(jointName)] = MOTOR(jointName)

    def Act(self, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(self.robotId, desiredAngle*c.motorJointRange)
    def Think(self):
        self.nn.Update()

    # def evaluate_h(self,points):

    #     # Compute the center of the starting points

    #     # Get initial starting points of all bots
    #     x_starts = []
    #     y_starts = []
    #     for i in points:
    #         trajectory = points[i]
    #         x_starts.append(trajectory[0,0])
    #         y_starts.append(trajectory[0,1])
            
    #         # Also create array of all coordinates for use in computing the bounding box around the trajectories later
    #         if i==0:
    #             all_coords = points[i]
    #         else:
    #             all_coords = np.concatenate((all_coords,points[i]))

    #     center_x = np.mean(x_starts)
    #     center_y = np.mean(y_starts)

    #     # Create a boundary of 20*constants.BOT_LENGTH
    #     min_x = center_x - c.BOUNDARY_LENGTH_X/2
    #     max_x = center_x + c.BOUNDARY_LENGTH_X/2
    #     min_y = center_y - c.BOUNDARY_LENGTH_Y/2
    #     max_y = center_y + c.BOUNDARY_LENGTH_Y/2

    #     # Count # unique points
    #     unique_points = np.unique(all_coords,axis=0)

    #     # Only keep unique points within the boundary

    #     # true if in bounds
    #     x_min_mask = np.reshape(unique_points[:,0]>min_x,newshape=(-1,1))
    #     x_max_mask = np.reshape(unique_points[:,0]<max_x,newshape=(-1,1))
    #     y_min_mask = np.reshape(unique_points[:,1]>min_y,newshape=(-1,1))
    #     y_max_mask = np.reshape(unique_points[:,1]<max_y,newshape=(-1,1))

    #     mask = np.all(np.concatenate((x_min_mask, x_max_mask, y_min_mask, y_max_mask),axis=1),axis=1)

    #     unique_points_in_bounds = unique_points[mask]

    #     return self.fractal_box_count(unique_points_in_bounds,(min_x,min_y,max_x,max_y))

    # def fractal_box_count(self, points, boundary):
    #     # https://francescoturci.net/2016/03/31/box-counting-in-numpy/

    #     min_x,min_y,max_x,max_y = boundary # unpack tuple

    #     Ns=[]

    #     # scales = np.arange(start=2, stop=int(constants.BOUNDARY_LENGTH/constants.MIN_GRID_DIM)) #start with quadrents and go to resolution of voxcraft float
    #     levels = np.arange(start=1, stop=c.MAX_LEVEL)

    #     for level in levels: 

    #         scale = 2**level

    #         cell_width = c.BOUNDARY_LENGTH_X/scale
    #         cell_height = c.BOUNDARY_LENGTH_Y/scale

    #         # H, edges=np.histogramdd(points, bins=(np.linspace(min_x,max_x,constants.BOUNDARY_LENGTH/scale),np.linspace(min_y,max_y,constants.BOUNDARY_LENGTH/scale)))
    #         H, edges=np.histogramdd(points, bins=(np.linspace(min_x,max_x,num=scale+1),np.linspace(min_y,max_y,num=scale+1)))

    #         weight = (cell_width*cell_height)/(c.BOUNDARY_LENGTH_X*c.BOUNDARY_LENGTH_Y) # David scaling
    #         Ns.append(np.sum(H>0)*weight)

    #     # Divide by # of levels to get a value between 0-1
    #     scaled_box_count = np.sum(Ns)/len(levels) # David scaling

    #     return scaled_box_count


    # def Get_Fitness(self):
    #     trajectories_arr = {}
    #     for i in self.trajectory:
    #         points_touples = self.trajectory[i]
    #         points_arr = np.reshape(points_touples, newshape=(len(points_touples),2))
    #         trajectories_arr[i] = points_arr
    #     fitness = self.evaluate_h(trajectories_arr)
    #     f = open("tmp{}.txt".format(self.solutionID), "w")
    #     f.write(str(fitness))
    #     f.close()
    #     os.system("ren tmp{}.txt fitness{}.txt".format(self.solutionID, self.solutionID))