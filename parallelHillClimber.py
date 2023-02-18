from solution import SOLUTION
import constants as c 
import copy 
import os 
class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvaliableID = 0
        self.parents = {}

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvaliableID)
            self.nextAvaliableID+=1 

    
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
        self.Print()
        self.Select()

    def Print(self):
        for k, v in self.parents.items():
            print(self.children[k].fitness, self.parents[k].fitness, "\n")

    def Spawn(self):
        self.children = {}
        for i, parent in self.parents.items():
            self.children[i] = copy.deepcopy(parent)
            self.children[i].Set_ID(self.nextAvaliableID)
            self.nextAvaliableID +=1 
   
    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Select(self):
        for k, v in self.children.items():
            if self.children[k].fitness < self.parents[k].fitness:
                self.parents[k] = self.children[k] 

    def Show_Best(self):
        best_fit = 10
        best_sln = ""
        for k, v in self.parents.items():
            if v.fitness < best_fit: 
                best_fit = v.fitness
                best_sln = v
        best_sln.Start_Simulation("x")

