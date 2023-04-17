import pybullet as p

physicsClient = p.connect(p.GUI)

# p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

# Define parameters of the sphere
radius = 0.1
mass = 1
start_pos = [0, 0, 1]

# Create collision shape
collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)

# Create multi-body with the collision shape
body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape, basePosition=start_pos)

for i in range(1000):
    p.stepSimulation()
# Set initial velocity and position
# p.resetBaseVelocity(body, [0, 0, 0])
# p.resetBasePosition(body, start_pos)