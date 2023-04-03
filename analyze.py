import numpy as np 
import matplotlib.pyplot as plt

# backLegSensorValues = np.load("data/backleg_sensor_val.npy")
# # with np.load("data/sensor_val.npz") as backLegSensorValues:

# print(backLegSensorValues)

# frontLegSensorValues = np.load("data/frontleg_sensor_val.npy")
# with np.load("data/sensor_val.npz") as backLegSensorValues:

f = open("fitness_evolution.csv", "r")
file_read = f.readlines()
cleaned_file = [float(ent.strip("\n")) for ent in file_read ]
plt.plot(cleaned_file, label = "Fitness Over Time")
plt.xlabel("Evolutionary Time")
plt.ylabel("Fitness")
# plt.plot(frontLegSensorValues, label = "Front leg")
plt.title("Fitness Over Time")
plt.legend()
plt.show()