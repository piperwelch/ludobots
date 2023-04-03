import constants as c
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

        maxForce = c.max_force)
    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.offset

        if self.jointName == b'Torso_Backleg':
            self.frequency *=2
        self.motorValues = np.sin(np.linspace(0, 2*3.14, c.simulation_time))
        for i in range(c.simulation_time):
            self.motorValues[i]= self.amplitude * np.sin(self.frequency * (i + self.offset))

        self.motorValues = np.interp(self.motorValues, (self.motorValues.min(), self.motorValues.max()), (-3.14/4, 3.14/4))
    def Save_Values(self):
        np.save("data/{}".format(self.jointName), self.motorValues)


