import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from numpy.linalg import norm

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
    box.paint_uniform_color([0, 0.7, 0])
    return box

def elastic_collision(pos, vel, radius=0.05):
    N = len(pos)
    mass = 1
    for i in range(N):
        for j in range(i + 1, N):
            rrel = pos[i] - pos[j]
            distance = norm(rrel)
            if distance < 2 * radius:  # Check if bodies are colliding
                vrel = vel[i] - vel[j]
                ratio0 = 2 * mass / (mass + mass) 
                ratio1 = 2 * mass / (mass + mass) 
                vel[i] = vel[i] - ratio0 * np.dot(vrel,rrel) / distance**2 *rrel 
                vel[j] = vel[j] - ratio1 * np.dot(-vrel,-rrel) / distance**2 *(-rrel)
    return vel

def move_and_bounce(pos, vel, acc, dt, d):
    # Update positions and velocities
    pos += vel * dt
    vel += acc * dt

    # Boundary check and bounce
    for i in range(3):  # x, y, z dimensions
        over = np.abs(pos[:, i]) > d
        vel[over, i] *= -1  # Reverse velocity for bodies hitting the boundary

    return pos, vel

def calculate_momentum(pos, vel):
    mass = 1
    p = np.sum(mass*vel,axis=0)
    return p

def calculate_kinetic_energy(pos, vel):
    mass = 1
    K = np.sum(1/2*mass*norm(vel)**2)
    return K


N_bodies = 100
size = 1.0
dt = 0.01
N_frames = 601

# Initial positions and velocities
pos = np.random.uniform(low=-1, high=1, size=(N_bodies, 3))
vel = np.random.uniform(low=-1, high=1, size=(N_bodies, 3))
acc = np.ones((N_bodies, 3)) *np.array((0,-9.81,0)) # Put acceleration here (e.g., gravity)
#acc = np.zeros((N_bodies, 3)) 

# Visualization setup
vis = o3d.visualization.Visualizer()
vis.create_window(width=768, height=768)
vis.add_geometry(create_box(size))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pos)
pcd.paint_uniform_color([0.4, 0.5, 0.6])
vis.add_geometry(pcd)

# Simulation loop
for index in range(N_frames):
    pos, vel = move_and_bounce(pos, vel, acc, dt, size)
    vel = elastic_collision(pos, vel)
    pcd.points = o3d.utility.Vector3dVector(pos)  # Update point cloud positions

    if index%100==0:
        counts, bins = np.histogram(vel)
        plt.stairs(counts, bins)
        plt.show()

    print('Momentum:',calculate_momentum(pos,vel),'Energy:', calculate_kinetic_energy(pos,vel) )


    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()


