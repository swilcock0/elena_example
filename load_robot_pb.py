import pybullet as p
import pybullet_data
import os
import sys
import time


# Connect to the physics server using SharedMemory GUI
# physicsClient = p.connect(p.GUI_SERVER)
# if physicsClient < 0:
#     physicsClient = p.connect(p.GUI) # Fall back to regular GUI if SharedMemory fails
#     print("Falling back to regular GUI mode")

class HideOutput(object):
    '''
    A context manager that block stdout for its scope, usage:

    with HideOutput():
        os.system('ls -l')
    '''
    DEFAULT_ENABLE = True
    def __init__(self, enable=None):
        if enable is None:
            enable = self.DEFAULT_ENABLE
        self.enable = enable
        if not self.enable:
            return
        sys.stdout.flush()
        self._origstdout = sys.stdout
        self._oldstdout_fno = os.dup(sys.stdout.fileno())
        self._devnull = os.open(os.devnull, os.O_WRONLY)

    def __enter__(self):
        if not self.enable:
            return
        self._newstdout = os.dup(1)
        os.dup2(self._devnull, 1)
        os.close(self._devnull)
        sys.stdout = os.fdopen(self._newstdout, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.enable:
            return
        sys.stdout.close()
        sys.stdout = self._origstdout
        sys.stdout.flush()
        os.dup2(self._oldstdout_fno, 1)
        os.close(self._oldstdout_fno) # Added

def run_shared_physics():
    import subprocess
    DETACHED_PROCESS = 8
    executable = "Bullet_shared_GUI.exe"
    subprocess.Popen(executable, creationflags=DETACHED_PROCESS)
    
def Disconnect():
    ''' Disconnect any and all instances of bullet '''
    with HideOutput():
        for i in range(100):
            try:
                p.disconnect(i)
            except:
                break # Don't whinge about non-existent servers   
    
    

    
    
    
# Disconnect any existing connections to Bullet
Disconnect()
# Run the shared physics server in a detached process
run_shared_physics()
time.sleep(1)
# Connect to the physics server using SharedMemory GUI
physicsClient = p.connect(p.SHARED_MEMORY)
 
    
    
    
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
print(robot_urdf)
robotBasePosition = [0, 0, 1]
robotBaseOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(robot_urdf, robotBasePosition, robotBaseOrientation)

# Get number of joints
num_joints = p.getNumJoints(robotId)

# Print joint information
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
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
target_positions = [0 for i in range(num_joints)]  # Example positions

# Control parameters
kp = 0.7  # Proportional gain
kv = 0.5  # Velocity gain
max_force = 10000  # Maximum force to apply
# Add joint damping to prevent oscillations
damping = 0.1  # Damping factor
for i in range(num_joints):
    p.changeDynamics(robotId, i, linearDamping=damping, angularDamping=1)
    
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