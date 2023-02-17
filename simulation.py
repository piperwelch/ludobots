from robot import ROBOT
from world import WORLD 
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random
import matplotlib.pyplot as plt
import math 
import constants as c 

class SIMULATION:

    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI
        physicsClient = 0
        if directOrGUI == "DIRECT":
            physicsClient = p.connect(p.DIRECT)
        else:
            physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        
        self.world = WORLD()
        self.robot = ROBOT(solutionID)

    def Run(self):
        for i in range(1000):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            if self.directOrGUI != "DIRECT":
                time.sleep(1/300)
    def Get_Fitness(self):
        self.robot.Get_Fitness()
    def __del__(self):
        p.disconnect()