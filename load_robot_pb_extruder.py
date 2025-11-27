import pybullet as p
import pybullet_data
import os
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
#robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation)  #vecchia creazione del robot
# Carica il robot con base fissa direttamente
robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation, useFixedBase=True)

# Carica l'estrusore (un cilindro semplice) come oggetto separato
extruder_urdf_path = os.path.join(current_dir, "extruder", "simple_extruder.urdf")
extruder_start_pos = [0, 0, 0]
extruder_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
extruder_id = p.loadURDF(extruder_urdf_path, extruder_start_pos, extruder_start_orientation, useFixedBase=False)


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



# Collega l'estrusore all'end-effector del robot con un constraint
end_effector_index = movable_joint_indices[-1]
end_effector_link_state = p.getLinkState(robotId, end_effector_index)
ee_pos = end_effector_link_state[0]
ee_orn = end_effector_link_state[1]

constraint_id = p.createConstraint(
    parentBodyUniqueId=robotId,
    parentLinkIndex=end_effector_index,
    childBodyUniqueId=extruder_id,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
    parentFrameOrientation=[0, 0, 0, 1],
    childFrameOrientation=[0, 0, 0, 1]
)

# Control parameters
kp = 0.1
kv = 0.8
max_force = 10000  # Maximum force to apply


#AGGIUNTO
# Trova l'indice dell'end-effector (di solito l'ultimo giunto mobile)
end_effector_index = movable_joint_indices[-1]+1
print("End-effector index:", end_effector_index)  # o scegli il giusto joint con getJointInfo


#creo la traiettoria
# Definizione dei punti della traiettoria (waypoints)
waypoints = [
    [0.3, 0.3, 0.3],  
    [1, 0.3, 0.3],  
    [1, 1, 0.3], 
    [1, 1, 1]  
]
# Parametri di traiettoria
points_per_segment = 300  # più è alto, più fluido è il movimento
# Funzione di interpolazione lineare
def interpolate_path(waypoints, points_per_segment):
    path = []
    for i in range(len(waypoints) - 1):
        start = np.array(waypoints[i])
        end = np.array(waypoints[i + 1])
        segment = np.linspace(start, end, points_per_segment)
        path.extend(segment)
    return path
# Calcolo della traiettoria completa dell'end-effector
trajectory = interpolate_path(waypoints, points_per_segment)
#pre-calcolo della traiettoria dei giunti
joint_trajectory = []
for point in trajectory:
    joint_positions = p.calculateInverseKinematics(robotId, end_effector_index, point)
    joint_trajectory.append(joint_positions)

trajectory_index = 0  # punto attuale nella traiettoria
# Impostazioni per visualizzare sfere ogni N passi
sphere_interval = 50
sphere_counter = 0
frame_skip = 50  # numero di frame da saltare prima di creare sfere
frame_counter = 0

#CREAZIONE SFERE
# FUNZIONE: crea una sfera nella posizione dell’end-effector
def create_sphere_at_position(position, radius=0.01):
    #sphere_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[0.4, 0.4, 0.4, 1])
    sphere_id = p.createMultiBody(baseMass=0,     #0=oggetto statico
                                  #baseCollisionShapeIndex=sphere_collision,
                                  baseVisualShapeIndex=sphere_visual,
                                  basePosition=position)
    # Disattiva del tutto la dinamica (per sicurezza extra)
    p.changeDynamics(sphere_id, -1, lateralFriction=0, spinningFriction=0, 
                  rollingFriction=0, linearDamping=0, angularDamping=0)
    return sphere_id



# Main simulation loop
while True:
    #calcola la posizione desiderata dell'end-effector
    target_xyz = trajectory[trajectory_index]
    trajectory_index = (trajectory_index + 1) % len(trajectory)
    #calcola le posizioni desiderate dei giunti
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
    

    if frame_counter > frame_skip and sphere_counter % 10 == 0:
        extruder_position = p.getBasePositionAndOrientation(extruder_id)[0]
        create_sphere_at_position(extruder_position)

    sphere_counter += 1
    frame_counter += 1


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