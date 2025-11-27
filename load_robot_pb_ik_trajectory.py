import pybullet as p
import pybullet_data
import os
import math
import time  #AGGIUNTO PER TIME
import numpy as np #per la traiettoria

# Connect to the physics server using SharedMemory GUI
physicsClient = p.connect(p.GUI_SERVER)
if physicsClient < 0:
    physicsClient = p.connect(p.GUI) # Fall back to regular GUI if SharedMemory fails
    print("Falling back to regular GUI mode")



# Get the current script directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Set the correct paths
urdf_path = os.path.join(current_dir, "crx_description", "urdf", "crx10ia_l")
mesh_path = os.path.join(current_dir, "crx_description", "meshes", "crx10ia_l")

# Add search paths
p.setAdditionalSearchPath(urdf_path)
p.setAdditionalSearchPath(mesh_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set up simulation
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

# Load the robot
robot_urdf = os.path.join(urdf_path, "crx10ia_l.urdf")
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

# Create a constraint to stick the robot to the plane
#constraintId = p.createConstraint(
#    parentBodyUniqueId=planeId,
#    parentLinkIndex=-1,
#    childBodyUniqueId=robotId,
#    childLinkIndex=-1,
#    jointType=p.JOINT_FIXED,
#    jointAxis=[0, 0, 0],
#    parentFramePosition=[0, 0, 0],
#    childFramePosition=[0, 0, 0],
#)

# Define target positions for joints (in radians)
#target_positions = [0 for i in range(num_joints)]  # Example positions

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

#creo la traiettoria
# Definizione dei punti della traiettoria (waypoints)
waypoints = [
    [0.3, 0.3, 0.3],  
    [1, 0.3, 0.3],  
    [1, 1, 0.3], 
    [1, 1, 1]  
]
# Parametri di traiettoria
points_per_segment = 100  # più è alto, più fluido è il movimento
# Funzione di interpolazione lineare
def interpolate_path(waypoints, points_per_segment):
    path = []
    for i in range(len(waypoints) - 1):
        start = np.array(waypoints[i])
        end = np.array(waypoints[i + 1])
        segment = np.linspace(start, end, points_per_segment)
        path.extend(segment)
    return path
# Calcolo della traiettoria completa
trajectory = interpolate_path(waypoints, points_per_segment)
trajectory_index = 0  # punto attuale nella traiettoria
print("Inizio simulazione")
print("Lunghezza traiettoria:", len(trajectory))


# Main simulation loop
while True:
    #AGGIUNTO PER CAPIRE SE IL CODICE FUNZIONA
    #if not p.getConnectionInfo()['isConnected']:
    #    print("Server disconnesso! Interrompo la simulazione.")
    #    break
    #AGGIUNTA PER LA TRAIETTORIA
    target_xyz = trajectory[trajectory_index]
    trajectory_index = (trajectory_index + 1) % len(trajectory)  # per farla ciclare all’infinito

    # Calcolo le posizioni dei giunti usando IK
    target_positions = p.calculateInverseKinematics(robotId, end_effector_index, target_xyz)
        
    # Apply position control to each joint
    for idx, joint_idx in enumerate(movable_joint_indices):
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[idx],
            positionGain=kp,
            velocityGain=kv,
            force=max_force
        )
    
    p.stepSimulation()
    
    # Get and print joint states
    joint_states = []
    for i in range(num_joints):
        state = p.getJointState(robotId, i)
        joint_states.append(state[0])  # state[0] is the current position
    # print(f"Current joint positions: {joint_states}")

    #AGGIUNTO
    # Pausa per non chiudere tutto subito
    time.sleep(1./240.)
#p.disconnect() #vecchia chiusura del ciclo