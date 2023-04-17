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
import math 
import pickle 
import matplotlib.pyplot as plt 
class ROBOT:

    def __init__(self, solutionID):
        with open("cilia_pickle0.p", 'rb') as f:
            self.voxel_ids, self.cilia_voxels, self.voxel_id_index  = pickle.load(f)
        self.solutionID = solutionID
        self.robotId = ""
        self.trajectory = {0:[]}
        self.length = 8
        while self.robotId == "":
            try:
                self.robotId = p.loadURDF("body{}.urdf".format(0))
            except:
                time.sleep(0.01)


        pyrosim.Prepare_To_Simulate(self.robotId)

        self.Prepare_To_Act()


        
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
        voxel_data = p.getLinkState(self.robotId, 1)
        voxel_com_pos = voxel_data[0]  
        self.trajectory[0].append((voxel_com_pos[0], voxel_com_pos[1]))


    def Prepare_To_Act(self):
        # with open("cilia_pool/pickle{}.p".format(self.solutionID), 'rb') as f:
        #     self.cilia  = pickle.load(f)[0]
        self.Gen_Cilia()

    def get_surface_cell_coords(self, arr):
        # Get the coordinates of cells on the surface of the bot
        xs = []
        ys = []
        zs = []
        neigh = self.get_neighbors(arr)

        for x in range(arr.shape[0]):
            for y in range(arr.shape[1]):
                for z in range(arr.shape[2]):
                    if arr[x,y,z]==1 and np.sum(neigh[x,y,z,:])<6:
                        xs.append(x)
                        ys.append(y)
                        zs.append(z)

        return xs,ys,zs
    

    def get_neighbors(self, a):
        b = np.pad(a, pad_width=1, mode='constant', constant_values=0)
        neigh = np.concatenate((
            b[2:, 1:-1, 1:-1, None], b[:-2, 1:-1, 1:-1, None],
            b[1:-1, 2:, 1:-1, None], b[1:-1, :-2, 1:-1, None],
            b[1:-1, 1:-1, 2:, None], b[1:-1, 1:-1, :-2, None]), axis=3)
        return neigh
        
    def Gen_Cilia(self):
        radius = 3
        length = radius * 2 

        self.body_as_array = np.zeros((length, length, length), dtype=int) #Generate the body shape 
        r2 = np.arange(-radius, radius ) ** 2 
        dist2 = r2[:, None, None] + r2[:, None] + r2
        self.body_as_array[dist2 < radius ** 2] = 1

        xs,ys,zs = self.get_surface_cell_coords(self.body_as_array)
        surface_array = np.zeros((self.length, self.length, self.length), dtype = int)
        surface_array[xs, ys, zs] = 1

        self.cilia = np.random.random(size=(self.length, self.length, self.length, 3))*400-200
        self.cilia[:,:,:,0][surface_array!=1] = 0 # clear uncecessary cilia #x cilia 
        self.cilia[:,:,:,1][surface_array!=1] = 0 #y cilia
        self.cilia[:,:,:,2] = 0 # no cilia forces in the z direction4
        # print(sum(sum(sum(sum(self.cilia)))))
        with open("cilia_pool/pickle{}.p".format(self.solutionID), "wb") as f: # "wb" because we want to write in binary mode
            pickle.dump([self.cilia], f)

        
    def fibonacci_sphere(self, samples=1000):

        xs = []
        ys = []
        zs = []
        phi = math.pi * (math.sqrt(5.) - 1.)  # golden angle in radians

        for i in range(samples):
            y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
            radius = math.sqrt(1 - y * y)  # radius at y

            theta = phi * i  # golden angle increment

            x = math.cos(theta) * radius
            z = math.sin(theta) * radius

            xs.append(x)
            ys.append(y)
            zs.append(z)

        return xs, ys, zs

    def Act(self, t):

        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # xs, ys, zs = [],[],[]
        # for pt in self.voxel_id_index.values():
        #     xs.append(pt[0])
        #     ys.append(pt[1])
        #     zs.append(pt[2])
        # print(type(zs[0]))
        # ax.scatter(xs, ys, zs)
        # # plt.scatter(0,0,10)
        # plt.show()

        for voxelID, c in self.voxel_ids.items():

            if int(c) != -1 and self.cilia_voxels[int(c)] == True:
                # time.sleep(10)
                voxel_data = p.getLinkState(self.robotId, int(c))
                voxel_com_pos = voxel_data[0]
                # print(c, voxel_com_pos)
                force = self.voxel_id_index[int(c)]
                        
                
                # xs, ys, zs = [],[],[]
                # for pt in self.voxel_id_index.values():
                #     xs.append(pt[0])
                #     ys.append(pt[1])
                #     zs.append(pt[2])
                # print(type(zs[0]))

                x_force = self.cilia[force[0], force[1], force[2], 0]
                y_force = self.cilia[force[0], force[1], force[2], 1]
                # print(x_force, y_force, voxel_com_pos)
                p.applyExternalForce(self.robotId, c, [x_force,y_force,0], voxel_com_pos, p.WORLD_FRAME)
                # print("force applied")


    def Think(self):
        self.nn.Update()
# for i in range(500):
#     rb = ROBOT(i)