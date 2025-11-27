import pybullet as p
import pybullet_data
import os
import time  #AGGIUNTO PER TIME
import numpy as np #per la traiettoria
import traceback

# Connect to the physics server using SharedMemory GUI
try:
    physicsClient = p.connect(p.GUI_SERVER)
    print("[DEBUG] Connected to PyBullet GUI_SERVER.")
    if physicsClient < 0:
        print("[DEBUG] GUI_SERVER connection failed, trying regular GUI...")
        physicsClient = p.connect(p.GUI)
        print("[DEBUG] Connected to PyBullet GUI.")
        print("Falling back to regular GUI mode")
except Exception as e:
    print("[ERROR] Failed to connect to PyBullet:", e)
    traceback.print_exc()
    raise

# Get the current script directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Set the correct paths
urdf_path = os.path.join(current_dir, "wsg_50_simulation", "urdf")
mesh_path = os.path.join(current_dir, "crx_description", "meshes", "crx10ia_l")

print(f"[DEBUG] Current script directory: {current_dir}")
print(f"[DEBUG] URDF path: {urdf_path}")
print(f"[DEBUG] Mesh path: {mesh_path}")

# Add search paths
try:
    p.setAdditionalSearchPath(urdf_path)
    p.setAdditionalSearchPath(mesh_path)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print("[DEBUG] Additional search paths set.")
except Exception as e:
    print("[ERROR] Failed to set additional search paths:", e)
    traceback.print_exc()
    raise

# Set up simulation
try:
    p.setGravity(0, 0, -10)
    print("[DEBUG] Gravity set.")
    planeId = p.loadURDF("plane.urdf")
    print(f"[DEBUG] Plane loaded with ID: {planeId}")
except Exception as e:
    print("[ERROR] Failed to set gravity or load plane:", e)
    traceback.print_exc()
    raise

cube_scale = 4  # scala di riduzione
cube_height = 0.08 * cube_scale  # dimensione originale × scala
cube_position = [0, 0, cube_height / 2]
cube_urdf_path = os.path.join(current_dir, "elements/cube.urdf")
print(f"[DEBUG] Cube URDF path: {cube_urdf_path}")
try:
    cube_id = p.loadURDF(cube_urdf_path, basePosition=cube_position, useFixedBase=True, globalScaling=cube_scale)
    print(f"[DEBUG] Cube loaded with ID: {cube_id}")
except Exception as e:
    print("[ERROR] Failed to load cube URDF:", e)
    traceback.print_exc()
    raise

# Load the robot
robot_urdf = os.path.join(urdf_path, "robot_with_gripper.urdf")
robotBasePosition = [0, 0, 0]
robotBaseOrientation = p.getQuaternionFromEuler([0, 0, 0])
#robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation)  #vecchia creazione del robot
# Carica il robot con base fissa direttamente
robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation, useFixedBase=True)
print(f"[DEBUG] Robot loaded with ID: {robotId}")

# Carica l'estrusore (un cilindro semplice) come oggetto separato
#extruder_urdf_path = os.path.join(current_dir, "wsg_50_simulation", "urdf", "wsg_50.urdf")  # <-- il tuo cono

# Get number of joints
try:
    num_joints = p.getNumJoints(robotId)
    print(f"[DEBUG] Number of joints in robot: {num_joints}")
except Exception as e:
    print("[ERROR] Failed to get number of joints:", e)
    traceback.print_exc()
    raise

# Print joint information 
#MODIFICATO PER TROVARE giunti mobili (POSITION_CONTROL) e salvaRE i loro indici
movable_joint_indices = []
for i in range(num_joints):
    try:
        joint_info = p.getJointInfo(robotId, i)
        joint_type = joint_info[2]
        joint_name = joint_info[1].decode('utf-8')
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            movable_joint_indices.append(i)
            print(f"[DEBUG] Movable Joint {i}: {joint_name}")
    except Exception as e:
        print(f"[ERROR] Failed to get info for joint {i}:", e)
        traceback.print_exc()

# Collega l'estrusore all'end-effector del robot con un constraint
try:
    end_effector_index = movable_joint_indices[-1]
    print(f"[DEBUG] End-effector index: {end_effector_index}")
    end_effector_link_state = p.getLinkState(robotId, end_effector_index)
    ee_pos = end_effector_link_state[0]
    ee_orn = end_effector_link_state[1]
    print(f"[DEBUG] End-effector position: {ee_pos}, orientation: {ee_orn}")
except Exception as e:
    print("[ERROR] Failed to get end-effector state:", e)
    traceback.print_exc()
    raise

# Control parameters
kp = 0.1
kv = 0.8
max_force = 10000  # Maximum force to apply

#creo la traiettoria
# Definizione dei punti della traiettoria (waypoints)
waypoints = [
    [1, 1, 1],  
    [1, 1, 0.3],  
    [1, 0.3, 0.3], 
    [0.3, 0.3, 0.3]  
]
# Parametri di traiettoria
points_per_segment = 500  # più è alto, più fluido è il movimento
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
print("[DEBUG] Creating trajectory...")
try:
    trajectory = interpolate_path(waypoints, points_per_segment)
    print(f"[DEBUG] Trajectory with {len(trajectory)} points created.")
    joint_trajectory = []
    for idx, point in enumerate(trajectory):
        if idx % 500 == 0:
            print(f"[DEBUG] Calculating IK for trajectory point {idx}/{len(trajectory)}: {point}")
        joint_positions = p.calculateInverseKinematics(robotId, end_effector_index, point)
        joint_trajectory.append(joint_positions)
    print(f"[DEBUG] Joint trajectory with {len(joint_trajectory)} points created.")
except Exception as e:
    print("[ERROR] Failed to create trajectory or calculate IK:", e)
    traceback.print_exc()
    raise

trajectory_index = 0  # punto attuale nella traiettoria
# Impostazioni per visualizzare sfere ogni N passi
sphere_interval = 50
sphere_counter = 0
frame_skip = 50  # numero di frame da saltare prima di creare sfere
frame_counter = 0

#CREAZIONE SFERE
# FUNZIONE: crea una sfera nella posizione dell'end-effector
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

last_had_contacts = False

# Utility function to get link index by name
def get_link_index_by_name(body_id, link_name):
    num_joints = p.getNumJoints(body_id)
    for i in range(num_joints):
        info = p.getJointInfo(body_id, i)
        if info[12].decode('utf-8') == link_name:
            return i
    raise ValueError(f"Link name '{link_name}' not found in body {body_id}")

# After loading the robot, get the index of the dummy_tcp link
try:
    dummy_tcp_index = get_link_index_by_name(robotId, "dummy_tcp")
    print(f"[DEBUG] dummy_tcp link index: {dummy_tcp_index}")
except Exception as e:
    print("[ERROR] Could not find dummy_tcp link:", e)
    traceback.print_exc()
    raise

# Main simulation loop
print("[DEBUG] Entering main simulation loop...")
while True:
    try:
        target_xyz = trajectory[trajectory_index]
        trajectory_index = (trajectory_index + 1) % len(trajectory)
        target_positions = p.calculateInverseKinematics(robotId, end_effector_index, target_xyz)
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
            extruder_position = p.getLinkState(robotId, dummy_tcp_index)[0]
            create_sphere_at_position(extruder_position)

        sphere_counter += 1
        frame_counter += 1

        p.stepSimulation()
        
        contacts = p.getContactPoints(bodyA=robotId, linkIndexA=dummy_tcp_index, bodyB=cube_id)
        had_contacts = len(contacts) > 0

        if had_contacts and not last_had_contacts:
             print(f"🚨 Collisione iniziata! Num contatti: {len(contacts)}")
             for contact in contacts:
                  print(f" - LinkA: {contact[3]}, LinkB: {contact[4]}, distanza: {contact[8]:.4f}")
        elif not had_contacts and last_had_contacts:
             print("✅ Collisione terminata.")

        last_had_contacts = had_contacts
        
        # Get and print joint states
        joint_states = []
        for i in range(num_joints):
            state = p.getJointState(robotId, i)
            joint_states.append(state[0])  # state[0] is the current position
        # print(f"Current joint positions: {joint_states}")

        #AGGIUNTO
        # Pausa per non chiudere tutto subito
        time.sleep(1./240.)
    except Exception as e:
        print("[ERROR] Exception in main simulation loop:", e)
        traceback.print_exc()
        break
#p.disconnect() #vecchia chiusura del ciclo