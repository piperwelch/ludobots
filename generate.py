import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5

def create_world():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x+30,y,z], size=[length, width, height]) 
    pyrosim.End()
    
def Generate_Body():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[1,0,1.5], size=[length, width, height]) 
    pyrosim.Send_Joint( name = "Torso_Backleg" , parent= "Torso" , child = "Backleg" , type = "revolute", position = [0.5,0,1])
    pyrosim.Send_Cube(name="Backleg", pos=[-0.5,0,-0.5], size=[length, width, height])
    pyrosim.Send_Joint( name = "Torso_Frontleg" , parent= "Torso" , child = "Frontleg" , type = "revolute", position = [1.5,0,1])
    pyrosim.Send_Cube(name="Frontleg", pos=[0.5,0,-0.5], size=[length, width, height]) 
    # pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0.5,0,1])
    # pyrosim.Send_Cube(name="Link3", pos=[0.5,0,0.5], size=[length, width, height])
    pyrosim.End()
def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "Backleg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "Frontleg")
    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_Backleg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_Frontleg")
    pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 3, weight = 1.0)
    pyrosim.End()   
create_world()
Generate_Body()
Generate_Brain()