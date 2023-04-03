import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random
import matplotlib.pyplot as plt
import math 
import constants as c 
from simulation import SIMULATION
import sys 

directOrGUI = sys.argv[1]
solutionID = int(sys.argv[2])
bot_ids = list(sys.argv[3].split(","))

simulation = SIMULATION(directOrGUI, solutionID, bot_ids)

simulation.Run()
simulation.Get_Fitness()


simulation.__del__()