import open3d as o3d
import numpy as np
from numpy.linalg import norm

BLUE = np.array((0.7,0.6,0.0))
YELLOW = np.array((0.0,0.6,0.8))

class Body:
    def __init__(self, radius, color, pos, vel, mass):
        self.radius = radius
        self.mesh = o3d.geometry.TriangleMesh.create_sphere(radius)
        self.mesh.compute_vertex_normals()
        self.mesh.paint_uniform_color(color)
        self.pos = pos    
        self.mesh.translate(pos)
        self.vel = vel
        self.acc = np.zeros(3)
        self.mass = mass

    def move(self,dt):
        dr = self.vel * dt
        self.mesh.translate(dr)
        self.pos = self.pos + dr    
         
def elastic_collision(b0,b1):
    vrel = b0.vel - b1.vel
    rrel = b0.pos - b1.pos
    distance = norm(rrel)
    ratio0 = 2 * b1.mass / (b0.mass + b1.mass) 
    ratio1 = 2 * b0.mass / (b0.mass + b1.mass) 
    b0.vel = b0.vel - ratio0 * np.dot(vrel,rrel) / distance**2 *rrel 
    b1.vel = b1.vel - ratio1 * np.dot(-vrel,-rrel) / distance**2 *(-rrel)

def inelastic_collision(b0,b1,dt = 0.01):
    # ball is working as a springs with frinction proportional to velocity                 
    K = 100 # elastic constant (N/m)
    B = 5 # damping
    vrel = b0.vel - b1.vel
    rrel = b0.pos - b1.pos
    distance =  norm(rrel)
    F = + K * rrel / distance * (b0.radius+b1.radius-distance)  - B * vrel
    acceleration0 = +F / b0.mass
    acceleration1 = -F / b1.mass
    b0.vel = b0.vel + acceleration0*dt
    b1.vel = b1.vel + acceleration1*dt    

def completely_inelastic_collision(b0,b1):
    m0 = b0.mass
    m1 = b1.mass
    p = m0 * b0.vel + m1 * b1.vel
    vm =  p /(m0+m1)
    b0.vel = vm
    b1.vel = vm

def calculate_kinetic_energy(b0,b1):
    K = 1/2 * b0.mass * norm(b0.vel)**2 + 1/2 * b1.mass * norm(b1.vel)**2 
    return(K)

def calculate_momentum(b0,b1):
    p0 = b0.mass * b0.vel
    p1 = b1.mass * b1.vel
    return (p0+p1)

body0 = Body(radius = 0.2,
             color = BLUE,
             pos = np.array((1.0,0.0,0.0)),
             vel = np.array((-1.0,0.0,0.0)),
             mass = 2.0
            ) 

body1 = Body(radius = 0.1,
             color = YELLOW,
             pos = np.array((-1.0,0.0,0.0)),
             vel = np.array((1.0,0.0,0.0)),
             mass = 1.0
            ) 

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

vis = o3d.visualization.Visualizer()
vis.create_window(width=768,height =768)
vis.add_geometry(frame)
vis.add_geometry(body0.mesh)
vis.add_geometry(body1.mesh)

N = 300 # number of frames in the movie
dt = 0.01

for i in range(N):

    body0.move(dt)
    body1.move(dt)

    distance = body0.pos - body1.pos
    if norm(distance) < (body0.radius+body1.radius):
        inelastic_collision(body0, body1)  
        
    print("Kinetic Energy:", calculate_kinetic_energy(body1, body0))
    #print("Momentum:      ", calculate_momentum(body1, body0))
    
    vis.update_geometry(body0.mesh)
    vis.update_geometry(body1.mesh)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()