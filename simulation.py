from robot import ROBOT
from world import WORLD 
import contextlib
with contextlib.redirect_stdout(None):
    import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random
import matplotlib.pyplot as plt
import math 
import constants as c 
from swarm import SWARM

class SIMULATION:

    def __init__(self, directOrGUI, solutionID, bot_ids):
        self.directOrGUI = directOrGUI
        self.solutionID = solutionID
        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD(self.solutionID)
        self.swarm = SWARM(self.solutionID, bot_ids)

    def Run(self):
        wall_length = 20


        # wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_length, 0.1, 1])
        # wall_body = p.createMultiBody(baseCollisionShapeIndex=wall_shape,
        #                             basePosition=[10, -10, 0])
        # wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_length, 0.1, 1])
        # wall_body = p.createMultiBody(baseCollisionShapeIndex=wall_shape,
        #                             basePosition=[10, 30, 0])
        

        # wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, wall_length, 1])
        # wall_body = p.createMultiBody(baseCollisionShapeIndex=wall_shape,
        #                             basePosition=[-10, 10, 0])
        # wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, wall_length, 1])
        # wall_body = p.createMultiBody(baseCollisionShapeIndex=wall_shape,
        #                             basePosition=[30, 10, 0])


        for i in range(c.simulation_time):
            p.stepSimulation()
            if i < 200:
                continue
            for robot in self.swarm.robots.values():
                # robot.Sense(i)
                # robot.Think()
                robot.Act(i)
                robot.Get_position()

            if self.directOrGUI != "DIRECT":
                time.sleep(1/300)

    def Get_Fitness(self):
        self.swarm.Get_Fitness()
        # plt.plot(self.swarm.robots[0].trajectory[0])

    def __del__(self):
        try:
            p.disconnect()
        except:
            pass