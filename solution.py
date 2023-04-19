import numpy as np
import pyrosim.pyrosim as pyrosim 
import os
import random 
import time
import constants as c
import matplotlib.pyplot as plt 
import pybullet as p 
from scipy.ndimage.measurements import center_of_mass
import pickle 
class SOLUTION:

    def __init__(self, myID, swarm_ids):
        self.myID = myID 
        self.bot_ids = swarm_ids
        self.VOXEL_SIZE = [0.1,0.1,0.1]

        self.radius = 3
        self.length = self.radius * 2 

        self.body_as_array = np.zeros((self.length, self.length, self.length), dtype=int) #Generate the body shape 
        r2 = np.arange(-self.radius, self.radius ) ** 2 
        dist2 = r2[:, None, None] + r2[:, None] + r2
        self.body_as_array[dist2 < self.radius ** 2] = 1
        self.Z_OFFSET = self.VOXEL_SIZE[0]/2 # body must be above ground
        self.voxel_IDs = {}
        self.next_voxel_ID_available = -1


    def Update_Weight_Map(self, id):
        count = 0
        keys = list(self.weights.keys())

        new_weights = self.weights.copy()
        for id in range(id, id+c.SWARM_SIZE):
            new_weights[id] = self.weights[keys[count]]
            count+=1 
        self.weights = new_weights

    def Create_Body(self, start_x, start_y, start_z, id, lockZ=True, save_filename="body0.urdf"):
        xs,ys,zs = self.get_surface_cell_coords(self.body_as_array)
        surface_array = np.zeros((self.length, self.length, self.length), dtype = int)
        surface_array[xs, ys, zs] = 1
        self.cilia_dict = {}
        self.voxel_id_index_map = {}
        pyrosim.Start_URDF("body{}.urdf".format(id))


        # compute COM of base layer (this will be the base joint if Z not locked)
        base_layer = self.body_as_array[:,:,1]

        base_com = center_of_mass(base_layer>0) # ignore cilia when computing center of mass

        base_link_pos = [int(base_com[0]), int(base_com[1]), 0]
        base_link_name = str(base_link_pos[1])+str(base_link_pos[1])+str(base_link_pos[2])

        # Add base voxel to the voxel_ID dictionary
        self.voxel_IDs[base_link_name] = self.next_voxel_ID_available
        self.next_voxel_ID_available+=1

        # Create base link at 0,0,0
        # voxel_color = "Yellow" if self.body_as_array[base_link_pos[0],base_link_pos[1],base_link_pos[2]]==2 else "Blue"

        pyrosim.Send_Cube(name=base_link_name, pos=[base_link_pos[0],base_link_pos[1],base_link_pos[2]], size=[0,0,0])

        voxels_generated = 1

        # Generate Links
        for x in range(self.body_as_array.shape[0]-1, 0, -1):
            for y in range(self.body_as_array.shape[1]-1, 0, -1):
                for z in range(int(self.body_as_array.shape[2]) -1, 0, -1):

                    voxel_name=str(x)+str(y)+str(z)
                    if self.body_as_array[x,y,z]>0 and voxel_name != base_link_name:
                        if surface_array[x,y,z] == 1:
                            self.cilia_dict[self.next_voxel_ID_available] = True
                            self.voxel_id_index_map[self.next_voxel_ID_available] = [x,y,z]
                        # else:
                        #     self.cilia_dict[self.next_voxel_ID_available] = False

                            self.voxel_IDs[voxel_name] = self.next_voxel_ID_available

                            self.next_voxel_ID_available+=1
                            
                            # voxel_color = "Yellow" if self.body_as_array[x,y,z]==2 else "Blue"
                        
                            # Send voxel
                            pyrosim.Send_Cube(name=voxel_name, pos=[start_y+y/(1/self.VOXEL_SIZE[0]),start_x+x/(1/self.VOXEL_SIZE[0]),z/(1/self.VOXEL_SIZE[0])+self.Z_OFFSET], size=self.VOXEL_SIZE)

                            voxels_generated+=1
            
        # Generate joints (to record information via getBasePositionAndOrientation joints have to be listed in the URDF after all links)
        
        # If lockZ connect base link to the actual fixed base
        # if lockZ:
        #     joint_name = "Swiveler_"+base_link_name
        #     pyrosim.Send_Joint(name=joint_name, parent="Swiveler", child=base_link_name, type="planar", position="0 0 0", jointAxis = "0 0 1")
        
        for x in range(self.body_as_array.shape[0]-1, 0, -1):
            for y in range(self.body_as_array.shape[1]-1, 0, -1):
                for z in range(int(self.body_as_array.shape[2]) - 1, 0, -1):
                    
                    voxel_name=str(x)+str(y)+str(z)
                    
                    if self.body_as_array[x,y,z]>0 and voxel_name != base_link_name:

                        # Connect to Base via fixed joint
                        if surface_array[x,y,z] == 1:

                            parent_name = base_link_name

                            joint_name = parent_name + "_" + voxel_name

                            pyrosim.Send_Joint(name=joint_name, parent=parent_name, child=voxel_name, type="fixed", position="0 0 0",jointAxis = "1 1 1")
        


        # cilia[:,:,:,0][surface_array!=1] = 0 # clear uncecessary cilia #x cilia 
        pyrosim.End()

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
    
    def Set_ID(self, id):
        self.myID = id 

    def Start_Simulation(self, directOrGUI):
        self.Create_World(0,0,0.5)
        start_locs = [[0, 0], [0,5], [5,0], [5,5]]
        count = 0
        for id in self.bot_ids:
            self.Create_Body(start_locs[count][0],start_locs[count][1],1,id)
            count+=1 

        not_finished = True

        while not_finished:
            with open("world{}.sdf".format(self.myID), "r") as file:
                last_line_w = file.readlines()[-1]

            if last_line_w == "</sdf>":
                not_finished = False             
        print(self.bot_ids)
        cm_arg = ",".join(str(id) for id in self.bot_ids)

        os.system("start /B python simulate.py {} {} {}".format(directOrGUI, self.myID, cm_arg))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness{}.txt".format(self.myID)):
            time.sleep(0.01)
        not_finished = True 
        while not_finished:
            try:
                f = open("fitness{}.txt".format(self.myID), "r")
                self.fitness = float(f.read())
                f.close()
                not_finished = False
            except: 
                time.sleep(0.01)

        for id in self.bot_ids:
            os.system("del fitness{}.txt".format(id))


    def Create_World(self, x, y, z):
        length = 1
        width = 0.1
      
        pyrosim.Start_SDF("world{}.sdf".format(self.myID))
        height = 1
        pyrosim.Send_Cube(name='x', pos=[30,30,1], size=[1,1,1])
        pyrosim.End()
    

    def Create_Brain(self, id):
        pass


    def Mutate(self, bot_pool):
        pass

# sln = SOLUTION(0, 0)
# sln.Create_Body(0,1,1,3)
# sln.Create_World(1,2,1)
# with open("cilia_pickle{}.p".format(0), "wb") as f: # "wb" because we want to write in binary mode
#     pickle.dump([sln.voxel_IDs, sln.cilia_dict, sln.voxel_id_index_map], f)
# print(sln.voxel_IDs, sln.cilia_dict)