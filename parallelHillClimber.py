from solution import SOLUTION
import constants as c 
import copy 
import os 
import time 
import numpy as np 
from glob import glob
class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        np.random.seed(0)
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvaliableID = 0
        self.parents = {}
        self.bot_pool = self.Gen_Bot_Pool()
        self.evo_file = open("fitness_evolution.csv", "w")
        
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvaliableID, list(np.random.choice(self.bot_pool, c.SWARM_SIZE)))
            self.nextAvaliableID+=1

    def Gen_Bot_Pool(self):
        pool_of_brains = glob("bot_pool/brain*")
        brain_ids = []
        for brain in pool_of_brains:
            if len(brain) == 20:
                brain_ids.append(int(brain[-6]))
            if len(brain) == 21:
                brain_ids.append(int(brain[-7:-5]))
            if len(brain) == 22:
                brain_ids.append(int(brain[-8:-5]))
        return brain_ids

    def Evolve(self, directOrGUI):
        self.directOrGUI = directOrGUI

        for currentGeneration in range(c.numberOfGenerations):
            self.Evaluate(self.parents)
            self.Evolve_For_One_Generation(self.directOrGUI)
        
    def Evaluate(self, solutions):

        for parent in solutions.values():
            parent.Start_Simulation(self.directOrGUI)
        for parent in solutions.values():
            parent.Wait_For_Simulation_To_End()

    def Evolve_For_One_Generation(self, directOrGUI):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        # self.Print()
        self.Record_Best()
        self.Select()
        

    def Print(self):
        for k, v in self.parents.items():
            print(self.children[k].fitness, self.parents[k].fitness, "\n")
    def Record_Best(self):
        
        best_fit = -10
        best_sln = ""
        for k, v in self.parents.items():
            if v.fitness > best_fit: 
                best_fit = v.fitness
                best_sln = v
        self.evo_file.write(str(best_fit) + "\n")
    def Spawn(self):
        self.children = {}
        for i, parent in self.parents.items():
            self.children[i] = copy.deepcopy(parent)
            self.children[i].Set_ID(self.nextAvaliableID)
            # self.children[i].Update_Weight_Map(self.nextAvaliableID)
            self.nextAvaliableID +=c.SWARM_SIZE
   
    def Mutate(self):
        for child in self.children.values():
            child.Mutate(self.bot_pool)

    def Select(self):
        for k, v in self.children.items():
            if self.children[k].fitness > self.parents[k].fitness:
                self.parents[k] = self.children[k] 

    def Show_Best(self):
        best_fit = -10
        best_sln = ""
        for k, v in self.parents.items():
            if v.fitness > best_fit: 
                best_fit = v.fitness
                best_sln = v
        best_sln.Start_Simulation("x")

    def Save_Best(self):
        time.sleep(2)
        best_fit = -10
        best_sln = ""
        best_sln_id = ""
        for k, v in self.parents.items():
            if v.fitness > best_fit: 
                best_fit = v.fitness
                best_sln = v
                best_sln_id = v.myID
            if float(v.fitness) > 3.0:
                os.system("move brain{}*.* bot_pool/".format(v.myID))
                os.system("move body{}*.* bot_pool/".format(v.myID))
        # f = open("best_sln.txt", "w")
        # f.write(str(best_sln.weights))
        
        for i in range(0, c.populationSize*(c.numberOfGenerations+1)):
            if i != int(best_sln_id):
                os.system("del brain{}.nndf".format(i))
                os.system("del body{}.urdf".format(i))
                os.system("del world{}.sdf".format(i))
    