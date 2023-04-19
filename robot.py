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
        self.length = 6
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
        # self.body = self.generate_sphere()
        # print(self.body)
        # self.cilia = self.generate_restricted_cilia_forces(self.body)
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
        
    def shift_down(self, body):
        while True:  # shift down until in contact with surface plane
            if np.sum(body[:, :, 0]) == 0:
                body[:, :, :-1] = body[:, :, 1:]
                body[:, :, -1] = np.zeros_like(body[:, :, -1])
            else:
                break
        return body

    def generate_sphere(self, radius=3):

        # make sphere 
        length = radius*2+1
        body = np.zeros((length, length, length), dtype=int)
        r2 = np.arange(-radius, radius + 1) ** 2
        dist2 = r2[:, None, None] + r2[:, None] + r2
        # body[dist2 < radius ** 2] = 1
        body[dist2 < radius ** 2] = 1

        return self.shift_down(body)

    def Gen_Cilia(self):
        # radius = 3
        # length = radius * 2 

        # self.body_as_array = np.zeros((length, length, length), dtype=int) #Generate the body shape 
        # r2 = np.arange(-radius, radius ) ** 2 
        # dist2 = r2[:, None, None] + r2[:, None] + r2
        # self.body_as_array[dist2 < radius ** 2] = 1

        # xs,ys,zs = self.get_surface_cell_coords(self.body_as_array)
        # surface_array = np.zeros((self.length, self.length, self.length), dtype = int)
        # surface_array[xs, ys, zs] = 1

        # self.cilia = (np.random.random(size=(self.length, self.length, self.length, 3))*2 - 1)*10
        # self.cilia[:,:,:,0][surface_array!=1] = 0 # clear uncecessary cilia #x cilia 
        # self.cilia[:,:,:,1][surface_array!=1] = 0 #y cilia
        # self.cilia[:,:,:,2] = 0 # no cilia forces in the z direction4
        # print(sum(sum(sum(sum(self.cilia)))))
        self.cilia = np.random.random(size=(92, 3))*150-75
        with open("cilia_pool/pickle{}.p".format(self.solutionID), "wb") as f: # "wb" because we want to write in binary mode
            pickle.dump([self.cilia], f)
        
        return self.cilia 

        
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
        #hard coded surface voxels for bot of radius  3 
        for voxelID in [-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 25, 26, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 43, 44, 48, 49, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 66, 67, 71, 72, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91]:
            
            xyz_forces = self.cilia[voxelID]
            xyz_forces[2] = 0



            if voxelID==-1:
                voxel_com_pos, voxel_com_orientation = p.getBasePositionAndOrientation(self.robotId)
            else:
                voxel_data = p.getLinkState(self.robotId, voxelID)
                voxel_com_pos = voxel_data[0]

            p.applyExternalForce(self.robotId, voxelID, xyz_forces, voxel_com_pos, p.WORLD_FRAME)
        # for voxelID, c in self.voxel_ids.items():
        #     if int(c) != -1 and self.cilia_voxels[int(c)] == True:
        #         # time.sleep(10)
        #         voxel_data = p.getLinkState(self.robotId, int(c))
        #         voxel_com_pos = voxel_data[0]
        #         # print(c, voxel_com_pos)
        #         force = self.voxel_id_index[int(c)]
                
        #         p.changeDynamics(self.robotId, c, lateralFriction=0.09)
        #         x_force = self.cilia[force[0], force[1], force[2], 0]
        #         y_force = self.cilia[force[0], force[1], force[2], 1]

        #         p.applyExternalForce(self.robotId, c, [x_force,y_force,0], [voxel_com_pos[0] + 0.05, voxel_com_pos[1] + 0.05,voxel_com_pos[2]], p.LINK_FRAME)
                # print("force applied")


    def Think(self):
        self.nn.Update()
for i in range(500):
    rb = ROBOT(i)