from solution import SOLUTION
import constants as c 
import copy 
class HILL_CLIMBER:

    def __init__(self):

        self.parent = SOLUTION()

    
    def Evolve(self, directOrGUI):
        self.parent.Evaluate("x")

        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(directOrGUI)
        self.Show_Best()
    
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

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.child.fitness > self.parent.fitness:
            self.parent = self.child 
    def Show_Best(self):
        self.parent.Evaluate("x")
