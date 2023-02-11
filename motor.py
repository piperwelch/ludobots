import constants
import pyrosim.pyrosim as pyrosim
import pybullet as p
import numpy as np
class MOTOR:

    def __init__(self, jointName):
        self.jointName = jointName 
        self.Prepare_To_Act()
    def Set_Value(self, robotId, desiredAngle):
        pyrosim.Set_Motor_For_Joint(

        bodyIndex = robotId,

        jointName = self.jointName,

        controlMode = p.POSITION_CONTROL,

        targetPosition = desiredAngle,

        maxForce = constants.max_force)
    def Prepare_To_Act(self):
        self.amplitude = constants.amplitude
        self.frequency = constants.frequency
        self.offset = constants.offset
        print("JOINT NAME", self.jointName)
        if self.jointName == b'Torso_Backleg':
            self.frequency *=2
        self.motorValues = np.sin(np.linspace(0, 2*3.14, 1000))
        for i in range(1000):
            self.motorValues[i]= self.amplitude * np.sin(self.frequency * (i + self.offset))

        self.motorValues = np.interp(self.motorValues, (self.motorValues.min(), self.motorValues.max()), (-3.14/4, 3.14/4))
    def Save_Values(self):
        np.save("data/{}".format(self.jointName), self.motorValues)


