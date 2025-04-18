import pybullet as p
import pybullet_data
import os
import sys
import time
from pybullet_utils import bullet_client as bc


physicsClient = bc.BulletClient(connection_mode=p.GUI)#p.connect(p.SHARED_MEMORY_SERVER)
    
    
# Get the current script directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Set the correct paths
urdf_path = os.path.join(current_dir, "crx10ia_l")
mesh_path = os.path.join(current_dir, "crx10ia_l", "meshes")

# Add search paths
p.setAdditionalSearchPath(urdf_path)
p.setAdditionalSearchPath(mesh_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set up simulation
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

# Load the robot
robot_urdf = os.path.join(urdf_path, "crx10ia_l.urdf")
print(robot_urdf)
robotBasePosition = [0, 0, 1]
robotBaseOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation)

# Get number of joints
num_joints = p.getNumJoints(robotId)

# Print joint information
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] == p.JOINT_REVOLUTE:
        print(f"Joint {i}: {joint_info[1].decode('utf-8')}")

# Create a constraint to stick the robot to the plane
constraintId = p.createConstraint(
    parentBodyUniqueId=planeId,
    parentLinkIndex=-1,
    childBodyUniqueId=robotId,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)

# Define target positions for joints (in radians)
target_positions = [0.0 for i in range(num_joints)]  # Example positions

# Control parameters
kp = 0.1  # Proportional gain
kv = 0.5  # Velocity gain
max_force = 100000  # Maximum force to apply
# Add joint damping to prevent oscillations
damping = 0.2  # Damping factor
for i in range(num_joints):
    p.changeDynamics(robotId, i, linearDamping=0, angularDamping=damping)
    
# Main simulation loop
while True:
    # Apply position control to each joint
    for i in range(num_joints):
        p.setJointMotorControl2(
            bodyIndex=robotId,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[i],
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
    
    # keys = p.getKeyboardEvents(physicsClient)
    # if ord('q') in keys:
    #     print("Exiting...")
    #     break
    # print(keys)
p.disconnect()