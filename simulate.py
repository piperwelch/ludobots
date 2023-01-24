import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random
import matplotlib.pyplot as plt
import math 
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
pi = 3.14159

backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)
amplitude_f, frequency_f, phaseOffset_f = 2*3.14/4, 1/80, 0
amplitude_b, frequency_b, phaseOffset_b = 3.14/4, 1/80,10
targetAngles_f = np.sin(np.linspace(0, 2*3.14, 1000))
targetAngles_b = np.sin(np.linspace(0, 2*3.14, 1000))
for i in range(1000):
    targetAngles_f[i]= amplitude_f * math.sin(frequency_f * (i + phaseOffset_f))
    targetAngles_b[i]= amplitude_b * math.sin(frequency_b * (i + phaseOffset_b))


# targetAngles_b = np.interp(targetAngles_b, (targetAngles_b.min(), targetAngles_b.max()), (-3.14/4, 3.14/4))
# targetAngles_f = np.interp(targetAngles_f, (targetAngles_f.min(), targetAngles_f.max()), (-3.14/4, 3.14/4))
# plt.plot(targetAngles_f)
# plt.plot(targetAngles_b)
# plt.show()
for i in range(1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Backleg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("Frontleg")
    pyrosim.Set_Motor_For_Joint(

    bodyIndex = robotId,

    jointName = b'Torso_Backleg',

    controlMode = p.POSITION_CONTROL,

    targetPosition = targetAngles_b[i],

    maxForce = 5)

    pyrosim.Set_Motor_For_Joint(

    bodyIndex = robotId,

    jointName = b'Torso_Frontleg',

    controlMode = p.POSITION_CONTROL,

    targetPosition = targetAngles_f[i],

    maxForce = 5)
    time.sleep(1/60)

print(backLegSensorValues)
plt.plot(backLegSensorValues)
plt.show()
np.save("data/backleg_sensor_val", backLegSensorValues)
np.save("data/frontleg_sensor_val", frontLegSensorValues)
p.disconnect()