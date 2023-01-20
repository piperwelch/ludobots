import numpy as np 
import matplotlib.pyplot as plt

backLegSensorValues = np.load("data/backleg_sensor_val.npy")
# with np.load("data/sensor_val.npz") as backLegSensorValues:

print(backLegSensorValues)

frontLegSensorValues = np.load("data/frontleg_sensor_val.npy")
# with np.load("data/sensor_val.npz") as backLegSensorValues:

print(frontLegSensorValues)
plt.plot(backLegSensorValues, label = "Back leg", linewidth=4)
plt.plot(frontLegSensorValues, label = "Front leg")
plt.title("Front and back leg sensor values")
plt.legend()
plt.show()