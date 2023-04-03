import numpy as np
import pyrosim.pyrosim as pyrosim 
import os
import random 
import time
import constants as c
class SOLUTION:

    def __init__(self, myID, swarm_ids):
        self.myID = myID 
        self.bot_ids = swarm_ids

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


    def Set_ID(self, id):
        self.myID = id 

    def Start_Simulation(self, directOrGUI):
        self.Create_World(0,0,0.5)
        start_locs = [[0, 0], [0,5], [5,0], [5,5]]
        count = 0
        for id in self.bot_ids:
            self.Create_Body(start_locs[count][0],start_locs[count][1],1,id)
            # self.Create_Brain(id)
            count+=1 

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
        width = 1
        height = 1
        x=10
        pyrosim.Start_SDF("world{}.sdf".format(self.myID))

        pyrosim.Send_Cube(name="Box", pos=[x,y,z], size=[5, width, height]) 
        # pyrosim.Send_Sphere(name="BowlingBall" , pos=[-3,+3,0.5] , size=[0.5])
        pyrosim.End()
    
    def Create_Body(self, start_x, start_y, start_z, id):
        length = 1
        width = 1
        height = 1

        pyrosim.Start_URDF("body{}.urdf".format(id))
        pyrosim.Send_Sphere(name="Torso", pos=[start_x, start_y, start_z], size=[1,1,1])

        # joints extending from root link
        pyrosim.Send_Joint(
            name="Torso_BackLeg", 
            parent="Torso", 
            child="BackLeg", 
            type="revolute", 
            position=[start_x, start_y-0.5,start_z], 
            jointAxis="1 0 1"
        )
        pyrosim.Send_Joint(
            name="Torso_LeftLeg", 
            parent="Torso", child="LeftLeg", 
            type="revolute", 
            position=[start_x-0.5, start_y, start_z], 
            jointAxis="0 1 1"
            )
        pyrosim.Send_Joint(
            name="Torso_RightLeg", 
            parent="Torso", 
            child="RightLeg", 
            type="revolute", 
            position=[start_x+0.5, start_y, start_z], 
            jointAxis="0 1 1"
            )
        pyrosim.Send_Joint(
            name="Torso_FrontLeg", 
            parent="Torso", 
            child="FrontLeg", 
            type="revolute", 
            position=[start_x, 
            start_y+0.5,start_z], 
            jointAxis="1 0 1"
            )

        # now all links & joints with an upstream joint have positions relative to the upstream joint
        pyrosim.Send_Joint(
            name="BackLeg_BackLower", 
            parent="BackLeg", 
            child="BackLower", 
            type="revolute", 
            position=[0,-1,0], 
            jointAxis="1 0 1"
            )
        pyrosim.Send_Joint(
            name="FrontLeg_FrontLower", 
            parent="FrontLeg", 
            child="FrontLower", 
            type="revolute", 
            position=[0,1,0], 
            jointAxis="1 0 1"
            )
        pyrosim.Send_Joint(
            name="RightLeg_RightLower", 
            parent="RightLeg", 
            child="RightLower", 
            type="revolute", 
            position=[1,0,0], 
            jointAxis="0 1 1"
            )
        pyrosim.Send_Joint(
            name="LeftLeg_LeftLower", 
            parent="LeftLeg", 
            child="LeftLower", 
            type="revolute", 
            position=[-1,0,0], 
            jointAxis="0 1 1"
            )

        # pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0.2,1,0.2])
        # pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0], size=[1,0.2,0.2])
        # pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0], size=[1,0.2,0.2])
        # pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0], size=[0,0,0])
        # pyrosim.Send_Cube(name="BackLower", pos=[0,0,-0.5], size=[0.2,0.2,1])
        # pyrosim.Send_Cube(name="FrontLower", pos=[0,0,-0.5], size=[0.2,0.2,1])
        # pyrosim.Send_Cube(name="LeftLower", pos=[0,0,-0.5], size=[0.2,0.2,1])
        # pyrosim.Send_Cube(name="RightLower", pos=[0,0,-0.5], size=[0.2,0.2,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0,0,0])
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0], size=[0,0,0])
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0], size=[0,0,0])
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0], size=[0,0,0])
        pyrosim.Send_Cube(name="BackLower", pos=[0,0,-0.5], size=[0,0,0])
        pyrosim.Send_Cube(name="FrontLower", pos=[0,0,-0.5], size=[0,0,0])
        pyrosim.Send_Cube(name="LeftLower", pos=[0,0,-0.5], size=[0,0,0])
        pyrosim.Send_Cube(name="RightLower", pos=[0,0,-0.5], size=[0,0,0])

        # pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0.5,0,1])
        # pyrosim.Send_Cube(name="Link3", pos=[0.5,0,0.5], size=[length, width, height])
        pyrosim.End()

    def Create_Brain(self, id):
        pyrosim.Start_NeuralNetwork("brain{}.nndf".format(id))
        pyrosim.Send_Sensor_Neuron(name=0, linkName="RightLower")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="LeftLower")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLower")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="BackLower")

        pyrosim.Send_Motor_Neuron(name=4, jointName=b'Torso_BackLeg')
        pyrosim.Send_Motor_Neuron(name=5, jointName=b'Torso_FrontLeg')
        pyrosim.Send_Motor_Neuron(name=6, jointName=b'Torso_LeftLeg')
        pyrosim.Send_Motor_Neuron(name=7, jointName=b'Torso_RightLeg')
        pyrosim.Send_Motor_Neuron(name=8, jointName=b'LeftLeg_LeftLower')
        pyrosim.Send_Motor_Neuron(name=9, jointName=b'RightLeg_RightLower')
        pyrosim.Send_Motor_Neuron(name=10, jointName=b'BackLeg_BackLower')
        pyrosim.Send_Motor_Neuron(name=11, jointName=b'FrontLeg_FrontLower')
        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3 , weight = self.weights[id][currentRow][currentColumn])

        pyrosim.End()  


    def Mutate(self):
        for id in range(self.myID, self.myID+ c.SWARM_SIZE):

            mutRow = random.randint(0,c.numSensorNeurons-1)
            mutCol = random.randint(0,c.numMotorNeurons-1)

            self.weights[id][mutRow, mutCol] = random.random() * 2 - 1
