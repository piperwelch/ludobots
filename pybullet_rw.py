from pyrosim import pyrosim 
import numpy as np
from scipy.ndimage.measurements import center_of_mass


class PYBULLET_RW():

    def __init__(self, body_as_array, cilia):

        self.VOXEL_SIZE = [1,1,1]
        self.Z_OFFSET = self.VOXEL_SIZE[0]/2 # body must be above ground

        self.body_as_array = body_as_array
        self.cilia = cilia

        self.voxel_IDs = {}
        self.next_voxel_ID_available = -1 # base index

    def write_body_urdf(self, lockZ=True, save_filename="body0.urdf"):
        self.surface_voxels_map = {}
        pyrosim.Start_URDF(save_filename)

        if lockZ:
            self.Generate_Fixed_Base()

            self.Generate_Swiveler()

            # self.Generate_Pusher()

        # compute COM of base layer (this will be the base joint if Z not locked)
        base_layer = self.body_as_array[:,:,1]
        base_com = center_of_mass(base_layer>0) # ignore cilia when computing center of mass
        print(base_com)
        base_link_pos = [int(base_com[0]), int(base_com[1]), 0]
        base_link_name = str(base_link_pos[1])+str(base_link_pos[1])+str(base_link_pos[2])

        # Add base voxel to the voxel_ID dictionary
        self.voxel_IDs[base_link_name] = self.next_voxel_ID_available
        self.next_voxel_ID_available+=1

        # Create base link at 0,0,0
        voxel_color = "Yellow" if self.body_as_array[base_link_pos[0],base_link_pos[1],base_link_pos[2]]==2 else "Blue"
        pyrosim.Send_Cube(name=base_link_name, pos=[base_link_pos[0],base_link_pos[1],base_link_pos[2]+self.Z_OFFSET], size=self.VOXEL_SIZE)

        voxels_generated = 1
        self.surface_voxels = np.zeros((length, length, length), dtype = int)
        xs,ys,zs = self.get_surface_cell_coords(self.body_as_array)
        self.surface_voxels[xs,ys,zs] =1
        # Generate Links
        for x in range(self.body_as_array.shape[0]):
            for y in range(self.body_as_array.shape[1]):
                for z in range(self.body_as_array.shape[2]):

                    voxel_name=str(x)+str(y)+str(z)

                    if self.body_as_array[x,y,z]>0 and voxel_name != base_link_name:

                        self.voxel_IDs[voxel_name] = self.next_voxel_ID_available

                        if self.surface_voxels[x,y,z] == 1:
                            self.surface_voxels_map[self.next_voxel_ID_available] = True 
                        self.next_voxel_ID_available+=1
                        
                        voxel_color = "Yellow" if self.body_as_array[x,y,z]==2 else "Blue"
                    
                        # Send voxel
                        pyrosim.Send_Cube(name=voxel_name, pos=[y,x,z+self.Z_OFFSET], size=self.VOXEL_SIZE)
                        

                        voxels_generated+=1
        
        # Generate joints (to record information via getBasePositionAndOrientation joints have to be listed in the URDF after all links)
        
        # If lockZ connect base link to the actual fixed base
        if lockZ:
            joint_name = "Swiveler_"+base_link_name
            pyrosim.Send_Joint(name=joint_name, parent="Swiveler", child=base_link_name, type="planar", position="0 0 0", jointAxis = "0 0 1")
        
        for x in range(self.body_as_array.shape[0]):
            for y in range(self.body_as_array.shape[1]):
                for z in range(self.body_as_array.shape[2]):
                    
                    voxel_name=str(x)+str(y)+str(z)

                    if self.body_as_array[x,y,z]>0 and voxel_name != base_link_name:

                        # Connect to Base via fixed joint
                        parent_name = base_link_name

                        joint_name = parent_name + "_" + voxel_name

                        pyrosim.Send_Joint(name=joint_name, parent=parent_name, child=voxel_name, type="fixed", position="0 0 0",  jointAxis = "0 0 1")

        pyrosim.End()
        print("Voxels Generated:", voxels_generated)

    def Generate_Fixed_Base(self):

        pyrosim.Send_Cube(name="FixedBase" , pos=[0,0,-1] , size=self.VOXEL_SIZE, mass=0.0)
        self.voxel_IDs["FixedBase"] = self.next_voxel_ID_available
        self.next_voxel_ID_available += 1

    def Generate_Swiveler(self):

        pyrosim.Send_Cube(  name="Swiveler" , pos=[0,0,-2] , size=self.VOXEL_SIZE, mass=0.0)

        pyrosim.Send_Joint( name = "FixedBase_Swiveler" , parent= "FixedBase" , child = "Swiveler" , type = "revolute", position = "0 0 0", jointAxis = "0 0 1")
        self.voxel_IDs["Swiveler"] = self.next_voxel_ID_available
        self.next_voxel_ID_available += 1

    # def Generate_Pusher(self):

    #     pyrosim.Send_Cube(  name="Pusher" , pos=[0,0,0] , size=self.VOXEL_SIZE)

    #     pyrosim.Send_Joint( name = "Swiveler_Pusher" , parent= "Swiveler" , child = "Pusher" , type = "planar", position = "0 0 0", jointAxis = "0 0 1")
    #     self.voxel_IDs["Pusher"] = self.next_voxel_ID_available
    #     self.next_voxel_ID_available += 1

    def write_cilia_txt(self, save_filename="data/cilia.txt"):
        print(self.voxel_IDs)

        f = open(save_filename,'w') 

        f.write('voxelName, voxelID, x, y, z\n')

        for x in range(self.body_as_array.shape[0]):
            for y in range(self.body_as_array.shape[1]):
                for z in range(self.body_as_array.shape[2]):

                    if self.body_as_array[x,y,z]==2:
                        voxel_name = str(x)+str(y)+str(z)

                        cilia_x = self.cilia[x,y,z][0]
                        cilia_y = self.cilia[x,y,z][1]
                        cilia_z = self.cilia[x,y,z][2]

                        f.write(voxel_name + ',' + str(self.voxel_IDs[voxel_name]) + ',' + str(cilia_x) + ',' + str(cilia_y) + ',' + str(cilia_z) + '\n')
        
        f.close()
    def get_surface_cell_coords(self, arr):
        # Get the coordinates of cells on the surface of the bot
        xs = []
        ys = []
        zs = []
        neigh = self.get_neighbors(arr)

        for x in range(arr.shape[0]):
            for y in range(arr.shape[1]):
                for z in range(arr.shape[2]):
                    if arr[x,y,z]==1 and np.sum(neigh[x,y,z,:])<6:
                        xs.append(x)
                        ys.append(y)
                        zs.append(z)

        return xs,ys,zs
    def get_neighbors(self, a):
        b = np.pad(a, pad_width=1, mode='constant', constant_values=0)
        neigh = np.concatenate((
            b[2:, 1:-1, 1:-1, None], b[:-2, 1:-1, 1:-1, None],
            b[1:-1, 2:, 1:-1, None], b[1:-1, :-2, 1:-1, None],
            b[1:-1, 1:-1, 2:, None], b[1:-1, 1:-1, :-2, None]), axis=3)
        return neigh
        
def generate_cilia_forces(body, lockZ):
    # Random forces array of size body.shape between [-1,1)
    # cilia = np.random.random(size=(body.shape[0], body.shape[1], body.shape[2], 3))*2-1
    cilia = np.random.random(size=(body.shape[0], body.shape[1], body.shape[2], 3))*40-20 # between [-10,10) - OK for lockZ
    # cilia = np.random.random(size=(body.shape[0], body.shape[1], body.shape[2], 3))*100-50 # between [-10,10)
    
    # Fixed cilia forces
    # cilia = np.ones((body.shape[0], body.shape[1], body.shape[2], 3))
    # cilia[:,:,:,0]=20
    # cilia[:,:,:,1]=20
    
    # Only assign forces to ciliated cells (material ID = 2)
    cilia[:,:,:,0][body!=2] = 0
    cilia[:,:,:,1][body!=2] = 0
    cilia[:,:,:,2] = 0 # no cilia forces in the z direction
    # if lockZ:
    #     cilia[:,:,:,2] = 0 # no cilia forces in the z direction
    # else:
    #     cilia[:,:,:,2][body!=2] = 0

    return cilia

def front(x,y,z,d=1):
    return x,y-d,z 

def back(x,y,z,d=1):
    return x,y+d,z

def left(x,y,z,d=1):
    return x-d,y,z

def right(x,y,z,d=1):
    return x+d,y,z

def get_empty_neighbor_positons(body,pos):
    empty_neigh = []
    for direction in [front,back,left,right]:
        neigh_pos = direction(pos[0],pos[1],pos[2])

        # Checking if the neighboring voxels is in array bounds
        if neigh_pos[0]>=0 and neigh_pos[0]<body.shape[0] and neigh_pos[1]>=0 and neigh_pos[1]<body.shape[1]:

            if body[neigh_pos] == 0: # in bounds and empty
                empty_neigh.append(neigh_pos)
    
        else: # out of array bounds so by default there is an empty neighbor
            empty_neigh.append(neigh_pos)
    
    return empty_neigh

def generate_restricted_cilia_forces(body, cilia_magnitude=50):
    # Assumes voxels do not have cilia forces in the z-direction

    cilia = np.zeros((body.shape[0], body.shape[1], body.shape[2], 3))

    rad45 = (45/180)*np.pi
    
    # iterate through ciliated cells
    for x in range(body.shape[0]):
        for y in range(body.shape[1]):
            for z in range(body.shape[2]):
                
                if body[x,y,z]==2:
                    curr_pos = (x,y,z)
                    # Get neighboring empty voxel locations
                    empty_neigh = get_empty_neighbor_positons(body,curr_pos)

                    # Compute vectors to directions of the empty neighbors
                    vectors = []
                    for empty_neigh_pos in empty_neigh:

                        # voxels can push only
                        x_comp = curr_pos[0]-empty_neigh_pos[0]
                        y_comp = curr_pos[1]-empty_neigh_pos[1]
                        z_comp = curr_pos[2]-empty_neigh_pos[2] # should always be 0
                        assert z_comp==0
                        
                        # by default all of the vectors are unit vectors because the distance between voxels is 1
                        vectors.append([x_comp,y_comp,z_comp])

                    # Compute range of angles the cilia force vector can lie in
                    # +/- 45 degrees of the vector to each empty neighboring voxel
                    if len(vectors)>0:
                        bounds = []

                        for vector in vectors:

                            angle_in_radians = np.arctan2(vector[1],vector[0])
                            lb = angle_in_radians-rad45 # lower bound
                            ub = angle_in_radians+rad45 # upper bound
                            
                            # convert angle to positive degrees around the unit circle if the angles are negative
                            print(lb,ub)
                            if lb < 0 and ub < 0: 
                                lb = 2*np.pi + lb
                                ub = 2*np.pi + ub
                                bounds.append([lb,ub])

                            elif lb<0 and ub > 0:
                                lb = 2*np.pi + lb
                                bounds.append([lb,np.pi*2])
                                bounds.append([0,ub])

                            elif lb>0 and ub<0:
                                ub = 2*np.pi + ub
                                bounds.append([ub,np.pi*2])
                                bounds.append([0,lb])

                            else:
                                bounds.append([lb,ub])
                        # print(bounds)

                        # first choose a random range to choose from if more than one 
                        # important if the ranges are not touching
                        # i.e. missing voxels to the left and right but not up and down
                        range_index = np.random.randint(len(bounds))
                        angle_range = bounds[range_index]

                        # Choose a random angle in the range (on the unit circle)
                        cilia_force_angle = (angle_range[1] - angle_range[0]) * np.random.random() + angle_range[0]
                        
                        # Compute the x and y components of the unit vector given the chosen angle
                        cilia_x_comp = np.sin(cilia_force_angle)
                        cilia_y_comp = np.cos(cilia_force_angle)
                        cilia_z_comp = 0
                        cilia_force_vec = [cilia_x_comp, cilia_y_comp, cilia_z_comp]                      
                        
                        print("CILIA UNIT VECTOR:",cilia_force_vec)

                        # multiple the vector specific magnitude so that it is large enough to make the bot move
                        cilia_force_vec = [x*cilia_magnitude for x in cilia_force_vec]
                        print("CILIA VECTOR:",cilia_force_vec)
                        
                        cilia[x,y,z,:] = cilia_force_vec
    return cilia

def shift_down(body):
    while True:  # shift down until in contact with surface plane
        if np.sum(body[:, :, 0]) == 0:
            body[:, :, :-1] = body[:, :, 1:]
            body[:, :, -1] = np.zeros_like(body[:, :, -1])
        else:
            break
    return body

def generate_sphere(radius=3):

    # make sphere 
    length = radius*2+1
    body = np.zeros((length, length, length), dtype=int)
    r2 = np.arange(-radius, radius + 1) ** 2
    dist2 = r2[:, None, None] + r2[:, None] + r2
    # body[dist2 < radius ** 2] = 1
    body[dist2 < radius ** 2] = np.random.choice([1,2],size=len(body[dist2 < radius ** 2]))

    return shift_down(body)

def generate_cube_array(cube_length_in_voxels=4):

    cube_array = np.ones((cube_length_in_voxels,)*3) # without cilia 
    # cube_array = np.random.randint(1,3,size=(cube_length_in_voxels,)*3) # with cilia
    # cube_array[3,2,3]=2 # top right middle voxel
    cube_array[3,2,3]=2

    # print(np.sum(cube_array==2))
    # print(np.sum(cube_array>0))

    return cube_array


radius = 3
length = radius * 2

body = np.zeros((length, length, length), dtype=int) #Generate the body shape 
r2 = np.arange(-radius, radius ) ** 2 
dist2 = r2[:, None, None] + r2[:, None] + r2
body[dist2 < radius ** 2] = 1
# while True:  # shift down until in contact with surface plane
#     if np.sum(body[:, :, 0]) == 0:
#         body[:, :, :-1] = body[:, :, 1:]
#         body[:, :, -1] = np.zeros_like(body[:, :, -1])
#     else:
#         break

body = generate_sphere()
prw = PYBULLET_RW(body, "x")

prw.write_body_urdf(0)

print(generate_restricted_cilia_forces(body))