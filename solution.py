import numpy as np
import pyrosim.pyrosim as pyrosim 
import os
import random 
class SOLUTION:

    def __init__(self, myID):
        self.myID = myID 
        self.weights = np.random.rand(3,2)
        self.weights = self.weights *2 -1
    def Set_ID(self, id):
        self.myID = id 
    def Evaluate(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

        os.system("start /B python simulate.py {} {}".format(directOrGUI, self.myID))
        f = open("fitness.txt", "r")
        self.fitness = float(f.read())

        f.close()
    def Create_World(self):
        length = 1
        width = 1
        height = 1
        x = 0
        y = 0
        z = 0.5
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[x+30,y,z], size=[length, width, height]) 
        pyrosim.End()
    
    def Create_Body(self):
        length = 1
        width = 1
        height = 1
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[1,0,1.5], size=[length, width, height]) 
        pyrosim.Send_Joint( name = "Torso_Backleg" , parent= "Torso" , child = "Backleg" , type = "revolute", position = [0.5,0,1])
        pyrosim.Send_Cube(name="Backleg", pos=[-0.5,0,-0.5], size=[length, width, height])
        pyrosim.Send_Joint( name = "Torso_Frontleg" , parent= "Torso" , child = "Frontleg" , type = "revolute", position = [1.5,0,1])
        pyrosim.Send_Cube(name="Frontleg", pos=[0.5,0,-0.5], size=[length, width, height]) 
        # pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0.5,0,1])
        # pyrosim.Send_Cube(name="Link3", pos=[0.5,0,0.5], size=[length, width, height])
        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain{}.nndf".format(self.myID))
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "Backleg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "Frontleg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = b'Torso_Backleg')
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = b'Torso_Frontleg')
        for currentRow in range(0,3):
            for currentColumn in range(0,2):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()   

    def Mutate(self):
        mutRow = random.randint(0,2)
        mutCol = random.randint(0,1)

        self.weights[mutRow, mutCol] = random.random() * 2 - 1
