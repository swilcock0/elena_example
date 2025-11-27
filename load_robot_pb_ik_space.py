#r: numpy
#r: matplotlib
#r: pybullet

import pybullet as p
import pybullet_data
import os
import math
import time  #AGGIUNTO PER TIME
import numpy as np #per la traiettoria
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scriptcontext as sc

def run_grasshopper_node(activate, grid_points, orientations):
    """
    Run the PyBullet simulation and store results in Grasshopper sticky dictionary.

    Args:
        activate (bool): Boolean input to trigger the simulation.
        grid_points (np.ndarray): Array of grid points (x, y, z).
        orientations (np.ndarray): Array of orientations (x, y, z).

    Returns:
        None
    """
    if not activate:
        return

    # Initialize PyBullet simulation
    physics_client = p.connect(p.DIRECT)  # Use DIRECT mode for non-graphical simulation
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set PyBullet data path
    p.setGravity(0, 0, -9.81)

    # Load robot and environment
    robot_id = p.loadURDF("robot_with_gripper.urdf", useFixedBase=True)
    end_effector_index = 7  # Replace with the actual index of the end effector

    def pybullet_collision_checker(point, orientation):
        """Check if a given point and orientation are reachable without collisions."""
        joint_positions = p.calculateInverseKinematics(robot_id, end_effector_index, point, orientation)
        for joint_index, joint_position in enumerate(joint_positions):
            p.resetJointState(robot_id, joint_index, joint_position)
        collision = p.getContactPoints(bodyA=robot_id)
        return len(collision) > 0

    # Test orientations for collisions
    collision_free_counts = test_orientations_for_collisions(
        grid_points, orientations, lambda point, orientation: not pybullet_collision_checker(
            point, p.getQuaternionFromEuler(orientation)
        )
    )

    # Store results in Grasshopper sticky dictionary
    sc.sticky["collision_free_counts"] = collision_free_counts
    sc.sticky["grid_points"] = grid_points

    # Disconnect PyBullet simulation
    p.disconnect()

# Connect to the physics server using SharedMemory GUI
physicsClient = p.connect(p.GUI_SERVER)
if physicsClient < 0:
    physicsClient = p.connect(p.GUI) # Fall back to regular GUI if SharedMemory fails
    print("Falling back to regular GUI mode")



# Get the current script directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Set the correct paths
urdf_path = os.path.join(current_dir, "wsg_50_simulation", "urdf")
mesh_path = os.path.join(current_dir, "crx_description", "meshes", "crx10ia_l")

# Add search paths
p.setAdditionalSearchPath(urdf_path)
p.setAdditionalSearchPath(mesh_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set up simulation
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

# Load the robot
urdf_path_old = os.path.join(current_dir, "crx_description", "urdf", "crx10ia_l")
robot_urdf = os.path.join(urdf_path_old, "crx10ia_l.urdf")
# robot_urdf = os.path.join(urdf_path, "robot_with_gripper.urdf")
# robot_urdf = os.path.join(urdf_path, "robot_with_gripper_and_base.urdf")
robotBasePosition = [0, 0, 0]
robotBaseOrientation = p.getQuaternionFromEuler([0, 0, 0])
#robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation) #vecchia creazione del robot
# Carica il robot con base fissa direttamente
robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation, useFixedBase=True)

# Get number of joints
num_joints = p.getNumJoints(robotId)

# Print joint information 
#MODIFICATO PER TROVARE giunti mobili (POSITION_CONTROL) e salvaRE i loro indici
movable_joint_indices = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_type = joint_info[2]
    joint_name = joint_info[1].decode('utf-8')
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        movable_joint_indices.append(i)
        print(f"Movable Joint {i}: {joint_name}")

joint_limits = []
for i in movable_joint_indices:
    info = p.getJointInfo(robotId, i)
    lower_limit = info[8]
    upper_limit = info[9]
    if lower_limit > upper_limit:  # in caso non siano definiti
        lower_limit, upper_limit = -np.pi, np.pi
    joint_limits.append((lower_limit, upper_limit))


# Control parameters
kp = 0.05  # Proportional gain
kv = 0.5  # Velocity gain
max_force = 10000  # Maximum force to apply
# Add joint damping to prevent oscillations
# damping = 1
# for i in range(num_joints):
#     p.changeDynamics(robotId, i, linearDamping=damping, angularDamping=damping)

#AGGIUNTO
# Trova l'indice dell'end-effector (di solito l'ultimo giunto mobile)
end_effector_index = movable_joint_indices[-1]
print("End-effector index:", end_effector_index)  # o scegli il giusto joint con getJointInfo
# Posizione desiderata della "mano" del robot (end-effector)
#target_xyz = [0.5, 0.2, 0.6] #PER ARRIVARE A QUESTA POSIZIONE DELLA MANO




# Campiona configurazioni e salva posizioni dell'end-effector
sampled_positions = []
n_samples = 10000

for _ in range(n_samples):
    # Genera una configurazione casuale entro i limiti
    joint_angles = [
        np.random.uniform(low, high)
        for (low, high) in joint_limits
    ]

    # Imposta lo stato dei giunti
    for idx, joint_idx in enumerate(movable_joint_indices):
        p.resetJointState(robotId, joint_idx, joint_angles[idx])

    # Ottieni posizione dell’end-effector
    pos = p.getLinkState(robotId, end_effector_index)[0]
    sampled_positions.append(pos)

# Estrai coordinate X, Y, Z
x_vals = [pos[0] for pos in sampled_positions if pos[2] > 0.0]
y_vals = [pos[1] for pos in sampled_positions if pos[2] > 0.0]
z_vals = [pos[2] for pos in sampled_positions if pos[2] > 0.0]
max_z = max(z_vals)
print(f"Max Z: {max_z}")

# Plot 3D del workspace
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_vals, y_vals, z_vals, s=1, c='blue', alpha=0.3)
ax.set_title('Workspace del braccio robotico')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
plt.show()


