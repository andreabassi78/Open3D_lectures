import open3d as o3d
import numpy as np
from numpy.linalg import norm

YELLOW = np.array((0.7,0.6,0.0))
BLUE = np.array((0.0,0.0,0.8))
GREEN = [0, 0.7, 0]

class Body:
    def __init__(self, radius, color, pos, vel, mass):
        self.radius = radius
        self.pos = pos    
        self.vel = vel
        self.acc = np.zeros(3)
        #self.acc = np.array((0.0,-9.81,0.0))
        self.mass = mass
        #self.mesh = o3d.geometry.TriangleMesh.create_sphere(radius)
        #self.mesh.paint_uniform_color(color)
        #self.mesh.translate(pos) 
        #self.mesh.compute_vertex_normals()

    def move(self,dt):
        dr = self.vel * dt
        self.pos += dr
        #self.mesh.translate(dr) 
        dv = self.acc *dt
        self.vel += dv

def elastic_collision(b0,b1):
    vrel = b0.vel - b1.vel
    rrel = b0.pos - b1.pos
    distance = norm(rrel)
    ratio0 = 2 * b1.mass / (b0.mass + b1.mass) 
    ratio1 = 2 * b0.mass / (b0.mass + b1.mass) 
    b0.vel = b0.vel - ratio0 * np.dot(vrel,rrel) / distance**2 *rrel 
    b1.vel = b1.vel - ratio1 * np.dot(-vrel,-rrel) / distance**2 *(-rrel)

def bounce(body,d):
    pos = body.pos
    vel = body.vel
    for i in range(3):  # x, y, z dimensions
        over = np.abs(pos[i]) > d
        body.vel[over, i] *= -1  # Reverse velocity for bodies hitting the boundary

def create_box(size):
    # Create a box represented by lines for visualization
    box = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([
            [-size, -size, -size], [size, -size, -size], 
            [-size, size, -size], [size, size, -size],
            [-size, -size, size], [size, -size, size], 
            [-size, size, size], [size, size, size]]),
        lines=o3d.utility.Vector2iVector([
            [0, 1], [0, 2], [1, 3], [2, 3],
            [4, 5], [4, 6], [5, 7], [6, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]))
    box.paint_uniform_color(GREEN)
    return box

vis = o3d.visualization.Visualizer()
vis.create_window(width=768,height =768)
box_size = 1
vis.add_geometry(create_box(size=box_size))

# Instantiate the molecules using the Body class

N_molecules = 3000
molecules = []
for index in range(N_molecules):
    pos = np.random.uniform(low=0.0, high=1.0, size=3) 
    vel = np.random.uniform(low=-1.0, high=1.0, size=3)
    molecule = Body(radius=0.00001, color=BLUE, pos=pos, vel=vel, mass=1.0)
    molecules.append(molecule)

# Rather then rendering each molecule we render a Point Cloud with their position xyz
xyz = np.zeros((N_molecules,3))
for index, molecule in enumerate(molecules):
    xyz[index,:] = molecule.pos
pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
pcd.paint_uniform_color(BLUE)
vis.add_geometry(pcd)

N_frames = 2500 # number of frames in the movie
dt = 0.01

for frame in range(N_frames):
    p = 0
    for i, molecule in enumerate(molecules): 
        molecule.move(dt)

        if molecule.pos[1]<-box_size:
            F = -2*molecule.mass*molecule.vel[1] /dt
            p += F /4/box_size**2

        bounce(molecule, d=box_size)
        # for j in range(i + 1, len(molecules)):
        #     other_molecule = molecules[j]
        #     r = molecule.pos - other_molecule.pos
        #     if norm(r) < (molecule.radius + other_molecule.radius):
        #         elastic_collision(molecule, other_molecule)
        xyz[i,:] = molecule.pos
    

    print("pressure:", p)
    pcd.points = o3d.utility.Vector3dVector(xyz)
    
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()