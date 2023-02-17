from solution import SOLUTION
import constants as c 
import copy 
class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        self.nextAvaliableID = 0
        self.parents = {}

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvaliableID )
            self.nextAvaliableID+=1 

    
    def Evolve(self, directOrGUI):
        for parent in self.parents.values():
            parent.Evaluate("DI")
        # self.parent.Evaluate("x")

        # for currentGeneration in range(c.numberOfGenerations):
        #     self.Evolve_For_One_Generation(directOrGUI)

    def Evolve_For_One_Generation(self, directOrGUI):
        self.Spawn()

        self.Mutate()

        self.child.Evaluate(directOrGUI)
        self.Print()
        self.Select()
    def Print(self):
        print(self.child.fitness, self.parent.fitness)
        
    def Spawn(self):
        self.child = copy.deepcopy(self.parent)
        self.child.Set_ID(self.nextAvaliableID)
        self.nextAvaliableID +=1 
    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.child.fitness > self.parent.fitness:
            self.parent = self.child 
    def Show_Best(self):
        pass
        # self.parent.Evaluate("x")
