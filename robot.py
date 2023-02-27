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
class ROBOT:

    def __init__(self, solutionID):
        self.solutionID = solutionID
        self.robotId = ""

        while self.robotId == "":
            try:
                self.robotId = p.loadURDF("body.urdf")
            except:
                time.sleep(0.01)


        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        self.nn = NEURAL_NETWORK("brain{}.nndf".format(self.solutionID))
        os.system("del brain{}.nndf".format(self.solutionID))


    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for k,v in self.sensors.items():
            v.Get_Value(t)

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


    def Get_Fitness(self):
       basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
       basePosition = basePositionAndOrientation[0]
       xPosition = basePosition[0]
       f = open("tmp{}.txt".format(self.solutionID), "w")
       f.write(str(xPosition))
       f.close()
       os.system("rename tmp{}.txt fitness{}.txt".format(self.solutionID, self.solutionID))



        
