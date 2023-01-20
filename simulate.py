import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np 

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

backLegSensorValues = np.zeros(100)
frontLegSensorValues = np.zeros(100)

for i in range(100):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Backleg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Frontleg")
    time.sleep(1/60)
    # print(i)
print(backLegSensorValues)
np.save("data/backleg_sensor_val", backLegSensorValues)
np.save("data/frontleg_sensor_val", frontLegSensorValues)
p.disconnect()