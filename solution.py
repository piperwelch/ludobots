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

        self.radius = 4
        self.length = self.radius * 2 

        self.body_as_array = np.zeros((self.length, self.length, self.length), dtype=int) #Generate the body shape 
        r2 = np.arange(-self.radius, self.radius ) ** 2 
        dist2 = r2[:, None, None] + r2[:, None] + r2
        self.body_as_array[dist2 < self.radius ** 2] = 1
        self.Z_OFFSET = self.VOXEL_SIZE[0]/2 # body must be above ground
        self.voxel_IDs = {}
        self.next_voxel_ID_available = -1
        # self.weights = {}
        # for id in range(myID, myID+ c.SWARM_SIZE):
        #     self.weights[id] = np.random.rand(c.numSensorNeurons,c.numSensorNeurons)
        #     self.weights[id] = self.weights[id] *2 -1

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

        if lockZ:
            pass
            # self.Generate_Fixed_Base()

            # self.Generate_Swiveler()

            # self.Generate_Pusher()

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
        print(base_link_pos)
        pyrosim.Send_Cube(name=base_link_name, pos=[base_link_pos[0],base_link_pos[1],base_link_pos[2]], size=[0,0,0])

        voxels_generated = 1

        # Generate Links
        for x in range(self.body_as_array.shape[0]-1, 0, -1):
            for y in range(self.body_as_array.shape[1]-1, 0, -1):
                for z in range(int(self.body_as_array.shape[2]) -2, 0, -1):

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
                            pyrosim.Send_Cube(name=voxel_name, pos=[y/(1/self.VOXEL_SIZE[0]),x/(1/self.VOXEL_SIZE[0]),z/(1/self.VOXEL_SIZE[0])+self.Z_OFFSET], size=self.VOXEL_SIZE)

                            voxels_generated+=1
            
        # Generate joints (to record information via getBasePositionAndOrientation joints have to be listed in the URDF after all links)
        
        # If lockZ connect base link to the actual fixed base
        # if lockZ:
        #     joint_name = "Swiveler_"+base_link_name
        #     pyrosim.Send_Joint(name=joint_name, parent="Swiveler", child=base_link_name, type="planar", position="0 0 0", jointAxis = "0 0 1")
        
        for x in range(self.body_as_array.shape[0]-1, 0, -1):
            for y in range(self.body_as_array.shape[1]-1, 0, -1):
                for z in range(int(self.body_as_array.shape[2]) - 2, 0, -1):
                    
                    voxel_name=str(x)+str(y)+str(z)
                    
                    if self.body_as_array[x,y,z]>0 and voxel_name != base_link_name:

                        # Connect to Base via fixed joint
                        if surface_array[x,y,z] == 1:

                            parent_name = base_link_name

                            joint_name = parent_name + "_" + voxel_name

                            pyrosim.Send_Joint(name=joint_name, parent=parent_name, child=voxel_name, type="fixed", position="0 0 0",jointAxis = "0 0 1")
        


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
            print(id,start_locs[count][0],start_locs[count][1])
            self.Create_Body(start_locs[count][0],start_locs[count][1],1,id)
            # self.Create_Brain(id)
            count+=1 
        # exit(0)
        # while not os.path.exists("brain{}.nndf".format(self.myID)):
        #     time.sleep(0.1)
        not_finished = True

        while not_finished:
            # with open("bot_pool/brain{}.nndf".format(self.myID), "r") as file:
            #     last_line_b = file.readlines()[-1]
            with open("world{}.sdf".format(self.myID), "r") as file:
                last_line_w = file.readlines()[-1]
            # if last_line_b == "</neuralNetwork>" and last_line_w == "</sdf>":
            #     not_finished = False  
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
    
    # def Create_Body(self, start_x, start_y, start_z, id):
    #     length = 1
    #     width = 1
    #     height = 1

    #     pyrosim.Start_URDF("body{}.urdf".format(id))
    #     radius = 4
    #     length = radius * 2

    #     body = np.zeros((length, length, length), dtype=int) #Generate the body shape 
    #     r2 = np.arange(-radius, radius ) ** 2 
    #     dist2 = r2[:, None, None] + r2[:, None] + r2
    #     body[dist2 < radius ** 2] = 1
    #     print(body.shape[0])
    #     for x in range(length):
    #         for y in range(length):
    #             for z in range(length):
    #                 if body[x,y,z] == 1:
    #                     pyrosim.Send_Cube(name="Box_{}_{}_{}".format(x,y,z), pos=[x/10,y/10,z/10], size=[0.1, 0.1, 0.1]) 
    #     for x in range(length):
    #         for y in range(length):
    #             for z in range(length):
    #                 try:
    #                     pyrosim.Send_Joint(name="{}_{}_{}_to_{}_{}_{}".format(x,y,z, x+1,y,z), 
    #                         parent="Box_{}_{}_{}".format(x,y,z), 
    #                         child="Box_{}_{}_{}".format(x+1,y,z), 
    #                         type="revolute", 
    #                         position=[x+0.5, y,z], 
    #                         jointAxis="1 0 1")
    #                     pyrosim.Send_Joint(name="{}_{}_{}_to_{}_{}_{}".format(x,y,z, x+1,y,z), 
    #                         parent="Box_{}_{}_{}".format(x,y,z), 
    #                         child="Box_{}_{}_{}".format(x,y+1,z), 
    #                         type="revolute", 
    #                         position=[x, y+0.5,z], 
    #                         jointAxis="1 0 1")
    #                     pyrosim.Send_Joint(name="{}_{}_{}_to_{}_{}_{}".format(x,y,z, x+1,y,z), 
    #                         parent="Box_{}_{}_{}".format(x,y,z), 
    #                         child="Box_{}_{}_{}".format(x,y,z+1), 
    #                         type="revolute", 
    #                         position=[x, y,z+0.5], 
    #                         jointAxis="1 0 1")
    #                 except:
    #                     continue
    #     # pyrosim.Send_Cube(name="Base", pos=[start_x, start_y, 0], size=[0.6,0.6,0.01])
    #     # pyrosim.Send_Joint(
    #     #     name="Torso_Base", 
    #     #     parent="Torso", 
    #     #     child="Base", 
    #     #     type="revolute", 
    #     #     position=[start_x, start_y,0.01], 
    #     #     jointAxis="1 0 1"
    #     # )
    #     pyrosim.End()

    def Create_Brain(self, id):
        pass
        # pyrosim.Start_NeuralNetwork("brain{}.nndf".format(id))
        # pyrosim.Send_Sensor_Neuron(name=0, linkName="RightLower")
        # pyrosim.Send_Sensor_Neuron(name=1, linkName="LeftLower")
        # pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLower")
        # pyrosim.Send_Sensor_Neuron(name=3, linkName="BackLower")

        # pyrosim.Send_Motor_Neuron(name=4, jointName=b'Torso_BackLeg')
        # pyrosim.Send_Motor_Neuron(name=5, jointName=b'Torso_FrontLeg')
        # pyrosim.Send_Motor_Neuron(name=6, jointName=b'Torso_LeftLeg')
        # pyrosim.Send_Motor_Neuron(name=7, jointName=b'Torso_RightLeg')
        # pyrosim.Send_Motor_Neuron(name=8, jointName=b'LeftLeg_LeftLower')
        # pyrosim.Send_Motor_Neuron(name=9, jointName=b'RightLeg_RightLower')
        # pyrosim.Send_Motor_Neuron(name=10, jointName=b'BackLeg_BackLower')
        # pyrosim.Send_Motor_Neuron(name=11, jointName=b'FrontLeg_FrontLower')
        # for currentRow in range(0,c.numSensorNeurons):
        #     for currentColumn in range(0,c.numMotorNeurons):
        #         pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3 , weight = self.weights[id][currentRow][currentColumn])

        # pyrosim.End()  


    def Mutate(self, bot_pool):
        pass
        # self.bot_ids[np.random.randint(len(self.bot_ids))] = np.random.choice(bot_pool)

        # for id in range(self.myID, self.myID+ c.SWARM_SIZE):

        #     mutRow = random.randint(0,c.numSensorNeurons-1)
        #     mutCol = random.randint(0,c.numMotorNeurons-1)

        #     self.weights[id][mutRow, mutCol] = random.random() * 2 - 1
sln = SOLUTION(0, 0)
sln.Create_Body(0,7,1,0)
sln.Create_World(1,2,1)
with open("cilia_pickle{}.p".format(0), "wb") as f: # "wb" because we want to write in binary mode
    pickle.dump([sln.voxel_IDs, sln.cilia_dict, sln.voxel_id_index_map], f)
print(sln.voxel_IDs, sln.cilia_dict)