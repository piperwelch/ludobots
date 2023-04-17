
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random
import matplotlib.pyplot as plt
import math 
import constants as c 
import time
import os 
class WORLD:

    def __init__(self, solutionID):
        time.sleep(1)
        p.loadSDF("world{}.sdf".format(0))
        planeId = p.loadURDF("plane.urdf")
        # os.system("del world{}.sdf".format(solutionID))