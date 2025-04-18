import pybullet as p
import pybullet_data
import os
import time
import itertools
from pybullet_utils import bullet_client as bc
import numpy as np
from deap import base, creator, tools, algorithms
import matplotlib.pyplot as plt
from multiprocessing import Pool
import json
from datetime import datetime
import signal

# Configuration Variables
SIMULATION_STEPS = 1000  # Number of steps for fitness evaluation
SETTLING_STEPS = 0    # Steps to let simulation settle
IMPULSE_PERIOD = 1000   # Steps between impulse applications in GUI simulation
BASE_IMPULSE_FORCE = 10000  # Base force magnitude for impulses
IMPULSE_DURATION = IMPULSE_PERIOD  # Force applies for the full period

# Grid Configuration
GRID_X_POSITIONS = [-2, 0, 2]
GRID_Y_POSITIONS = [-2, 2]

# Physics Configuration
GRAVITY = [0, 0, -10]
PHYSICS_PARAMS = {
    "numSolverIterations": 10,
    "contactBreakingThreshold": 0.001,
    "enableFileCaching": 0,
    "deterministicOverlappingPairs": 0
}

# Genetic Algorithm Configuration
POPULATION_SIZE = 10 
NUM_GENERATIONS = 30 
CROSSOVER_PROB = 0.5
MUTATION_PROB = 0.4
TOURNAMENT_SIZE = 3
MUTATION_RATE = MUTATION_PROB  # Mutation rate for each parameter

# Control Parameter Ranges
GAIN_MIN = 0.0
GAIN_MAX = 5.0
FORCE_MIN = 0  # For logarithmic scale: e^0
FORCE_MAX = 6  # For logarithmic scale: e^6

# Debug Configuration
DEBUG_LINE_COLOR = [1, 0, 0]  # Red
DEBUG_LINE_WIDTH = 1
DEBUG_LINE_LIFETIME = 0.1

# Simulation timestep
SIMULATION_TIMESTEP = 1./240.

# Random force range for GUI simulation
RANDOM_FORCE_MIN = -150
RANDOM_FORCE_MAX = 150

# Number of parallel simulation environments
NUM_PARALLEL_ENVIRONMENTS = 8

def spawn_robot_grid():
    """Create multiple parallel simulation environments"""
    clients_data = []
    all_robots_data = []
    
    for _ in range(NUM_PARALLEL_ENVIRONMENTS):
        # Create a DIRECT client for each environment
        client = bc.BulletClient(connection_mode=p.DIRECT)
        
        # Get paths
        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(current_dir, "crx10ia_l")
        mesh_path = os.path.join(current_dir, "crx10ia_l", "meshes")
        
        # Setup client
        client.setAdditionalSearchPath(urdf_path)
        client.setAdditionalSearchPath(mesh_path)
        client.setAdditionalSearchPath(pybullet_data.getDataPath())
        client.setGravity(*GRAVITY)
        client.setRealTimeSimulation(0)
        client.setPhysicsEngineParameter(**PHYSICS_PARAMS)
        
        # Load plane and robots for this environment
        plane_id = client.loadURDF("plane.urdf")
        robots_data = []
        
        # Load robots in grid
        robot_urdf = os.path.join(urdf_path, "crx10ia_l.urdf")
        grid_positions = list(itertools.product(GRID_X_POSITIONS, GRID_Y_POSITIONS))
        
        for x, y in grid_positions:
            robotBasePosition = [x, y, 0]
            robotBaseOrientation = client.getQuaternionFromEuler([0, 0, 0])
            robotId = client.loadURDF(
                robot_urdf,
                robotBasePosition,
                robotBaseOrientation
            )
            
            # Disable collisions
            for link in range(client.getNumJoints(robotId)):
                client.setCollisionFilterPair(robotId, plane_id, link, -1, 0)
                for other_link in range(link):
                    client.setCollisionFilterPair(robotId, robotId, link, other_link, 0)
            
            robots_data.append((client, robotId, plane_id))
        
        clients_data.append(client)
        all_robots_data.append(robots_data)
    
    return clients_data, all_robots_data, grid_positions

clients_data, all_robots_data, grid_positions = spawn_robot_grid()

print("\n\n")


def generate_impulse_forces(grid_positions):
    """Generate different impulse forces for each robot position"""
    forces = []
    for x, y in grid_positions:
        # Scale force based on position
        scale_x = 1 + x/2  # Scale up with distance from center
        scale_y = 1 + y/2
        force = [
            BASE_IMPULSE_FORCE * scale_x,  # X component
            BASE_IMPULSE_FORCE * scale_y,  # Y component
            BASE_IMPULSE_FORCE            # Z component always constant
        ]
        forces.append(force)
    return forces

def create_base_constraints(all_robots_data, grid_positions):
    """Create fixed constraints for all robots in all environments"""
    all_constraints = []
    
    # Iterate through each environment
    for robots_data in all_robots_data:
        # Create constraints for each robot in this environment
        for (client, robot_id, plane_id), (x, y) in zip(robots_data, grid_positions):
            constraint = client.createConstraint(
                parentBodyUniqueId=plane_id,
                parentLinkIndex=-1,
                childBodyUniqueId=robot_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[x, y, 0],
                childFramePosition=[0, 0, 0]
            )
            all_constraints.append(constraint)
    return all_constraints

constraints = create_base_constraints(all_robots_data, grid_positions)

def get_tcp_position(client, robot_id):
    """Get end-effector position for a robot"""
    num_joints = client.getNumJoints(robot_id)
    # Get the link state of the last joint (TCP)
    link_state = client.getLinkState(robot_id, num_joints - 1)
    return np.array(link_state[0])  # Position vector

initial_tcp_positions = []
tcp_position_history = []

# Initialize for all robots in all environments
for robots_data in all_robots_data:
    env_positions = []
    env_history = []
    for client, robot_id, _ in robots_data:
        initial_pos = get_tcp_position(client, robot_id)
        env_positions.append(initial_pos)
        env_history.append([])
    initial_tcp_positions.append(env_positions)
    tcp_position_history.append(env_history)

def fitness_function(individual):
    """Evaluate fitness using parallel environments"""
    kp, kv, max_force = individual
    all_deviations = []
    
    # Reset all environments at once
    for env_idx, robots_data in enumerate(all_robots_data):
        for (client, robot_id, _), (x, y) in zip(robots_data, grid_positions):
            client.resetBasePositionAndOrientation(
                robot_id,
                posObj=[x, y, 0],
                ornObj=client.getQuaternionFromEuler([0, 0, 0])
            )
            # Reset each joint individually
            for joint in range(client.getNumJoints(robot_id)):
                client.resetJointState(robot_id, joint, targetValue=0)
    
    # Let simulation settle
    for _ in range(SETTLING_STEPS):
        for robots_data in all_robots_data:
            for client, _, _ in robots_data:
                client.stepSimulation()
    
    # Batch simulate all environments
    for step in range(SIMULATION_STEPS):
        # Generate new forces at regular intervals
        if step % IMPULSE_PERIOD == 0:
            impulse_forces = []
            for env_idx, robots_data in enumerate(all_robots_data):
                env_forces = []
                for (client, robot_id, _), (x, y) in zip(robots_data, grid_positions):
                    # Scale force based on position
                    scale_x = 1 + x/2
                    scale_y = 1 + y/2
                    force = [
                        BASE_IMPULSE_FORCE * scale_x,
                        BASE_IMPULSE_FORCE * scale_y,
                        BASE_IMPULSE_FORCE
                    ]
                    env_forces.append(force)
                impulse_forces.append(env_forces)

        # Apply current forces every step
        for env_idx, robots_data in enumerate(all_robots_data):
            for robot_idx, (client, robot_id, _) in enumerate(robots_data):
                current_pos = get_tcp_position(client, robot_id)
                client.applyExternalForce(
                    robot_id,
                    linkIndex=client.getNumJoints(robot_id) - 1,
                    forceObj=impulse_forces[env_idx][robot_idx],
                    posObj=current_pos,
                    flags=client.WORLD_FRAME
                )
        
        # Rest of simulation step remains the same
        for env_idx, robots_data in enumerate(all_robots_data):
            step_deviations = []
            
            # Batch control commands for all robots in this environment
            for robot_idx, (client, robot_id, _) in enumerate(robots_data):
                client.setJointMotorControlArray(
                    robot_id,
                    range(client.getNumJoints(robot_id)),
                    controlMode=p.POSITION_CONTROL,
                    targetPositions=[0] * client.getNumJoints(robot_id),
                    positionGains=[kp] * client.getNumJoints(robot_id),
                    velocityGains=[kv] * client.getNumJoints(robot_id),
                    forces=[max_force] * client.getNumJoints(robot_id)
                )
            
            # Step simulation for all robots in this environment at once
            client.stepSimulation()
            
            # Collect results
            for robot_idx, (client, robot_id, _) in enumerate(robots_data):
                current_pos = get_tcp_position(client, robot_id)
                tcp_position_history[env_idx][robot_idx].append(current_pos)
                deviation = np.linalg.norm(current_pos - initial_tcp_positions[env_idx][robot_idx])
                step_deviations.append(deviation)
            
            all_deviations.extend(step_deviations)
    
    avg_deviation = np.mean(all_deviations)
    weighted_fitness = 0.7 * max(all_deviations) + 0.3 * avg_deviation
    return weighted_fitness,

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()

def generate_gain():
    return np.random.uniform(GAIN_MIN, GAIN_MAX)

def generate_force():
    return np.exp(np.random.uniform(FORCE_MIN, FORCE_MAX))

toolbox.register("kp", generate_gain)
toolbox.register("kv", generate_gain)
toolbox.register("force", generate_force)

def create_individual():
    return [
        generate_gain(),
        generate_gain(),
        generate_force()
    ]

toolbox.register("individual", tools.initIterate, creator.Individual, create_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("evaluate", fitness_function)
toolbox.register("mate", tools.cxTwoPoint)

def custom_mutate(individual):
    generators = [generate_gain, generate_gain, generate_force]
    for i, gen in enumerate(generators):
        if np.random.random() < MUTATION_RATE:
            individual[i] = gen()
    return individual,

toolbox.register("mutate", custom_mutate)
toolbox.register("select", tools.selTournament, tournsize=TOURNAMENT_SIZE)

# Register statistics
stats = tools.Statistics(lambda ind: ind.fitness.values)
stats.register("avg", np.mean)
stats.register("std", np.std)
stats.register("min", np.min)
stats.register("max", np.max)

if __name__ == '__main__':
    pool = None
    gui_client = None
    
    try:
        # Ignore SIGINT before creating pool
        original_sigint_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        pool = Pool(processes=NUM_PARALLEL_ENVIRONMENTS)
        # Restore SIGINT handler
        signal.signal(signal.SIGINT, original_sigint_handler)
        
        # Register pool map with toolbox
        toolbox.register("map", pool.map)
        
        # Initialize statistics
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", np.mean)
        stats.register("std", np.std)
        stats.register("min", np.min)
        stats.register("max", np.max)
        
        # Create Hall of Fame
        hof = tools.HallOfFame(1)
        
        # Run evolution directly (not in another process)
        pop = toolbox.population(n=POPULATION_SIZE)
        try:
            result, logbook = algorithms.eaSimple(
                pop, toolbox, 
                cxpb=CROSSOVER_PROB, 
                mutpb=MUTATION_PROB, 
                ngen=NUM_GENERATIONS,
                stats=stats,
                halloffame=hof,
                verbose=True
            )
        except KeyboardInterrupt:
            print("\nEvolution interrupted by user")
            result = pop
            raise
            
        # Extract and plot statistics
        gen = logbook.select("gen")
        fit_mins = logbook.select("min")
        fit_avgs = logbook.select("avg")
        fit_stds = logbook.select("std")
        
        # Create the performance plot with error bars
        plt.figure(figsize=(10, 6))
        plt.errorbar(gen, fit_avgs, yerr=fit_stds, label='Avg ± Std')
        plt.plot(gen, fit_mins, 'g-', label='Best Fitness')
        plt.xlabel('Generation')
        plt.ylabel('Fitness (lower is better)')
        plt.title('Genetic Algorithm Performance')
        plt.legend(loc='best')
        plt.grid(True)
        
        # Save and show the plot
        plt.savefig('ga_performance.png')
        plt.show()

        best_individual = tools.selBest(result, k=1)[0]
        best_kp, best_kv, best_max_force = best_individual

        print(f"Best parameters found:")
        print(f"kp: {best_kp}")
        print(f"kv: {best_kv}")
        print(f"max_force: {best_max_force}")

        # Prepare results dictionary
        results = {
            "parameters": {
                "kp": float(best_kp),
                "kv": float(best_kv),
                "max_force": float(best_max_force)
            },
            "performance": {
                "generations": list(gen),
                "best_fitness": list(map(float, fit_mins)),
                "avg_fitness": list(map(float, fit_avgs)),
                "std_fitness": list(map(float, fit_stds))
            },
            "config": {
                "population_size": POPULATION_SIZE,
                "num_generations": NUM_GENERATIONS,
                "crossover_prob": CROSSOVER_PROB,
                "mutation_prob": MUTATION_PROB,
                "timestamp": datetime.now().isoformat()
            }
        }
        
        # Save to JSON file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"ga_results_{timestamp}.json"
        with open(filename, 'w') as f:
            json.dump(results, f, indent=4)
        print(f"\nResults saved to {filename}")

        kp = best_kp
        kv = best_kv
        max_force = best_max_force

        print("Starting GUI simulation with best parameters:")
        print(f"kp: {kp:.3f}, kv: {kv:.3f}, max_force: {max_force:.3f}")

        gui_client = bc.BulletClient(connection_mode=p.GUI)
        gui_client.setRealTimeSimulation(0)
        gui_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(current_dir, "crx10ia_l")
        mesh_path = os.path.join(current_dir, "crx10ia_l", "meshes")

        gui_client.setAdditionalSearchPath(urdf_path)
        gui_client.setAdditionalSearchPath(mesh_path)
        gui_client.setAdditionalSearchPath(pybullet_data.getDataPath())

        gui_client.setGravity(*GRAVITY)
        gui_client.setPhysicsEngineParameter(**PHYSICS_PARAMS)

        plane_id = gui_client.loadURDF("plane.urdf")
        robot_urdf = os.path.join(urdf_path, "crx10ia_l.urdf")
        robotId = gui_client.loadURDF(robot_urdf, [0, 0, 0], gui_client.getQuaternionFromEuler([0, 0, 0]))
        num_joints = gui_client.getNumJoints(robotId)

        constraint = gui_client.createConstraint(
            parentBodyUniqueId=plane_id,
            parentLinkIndex=-1,
            childBodyUniqueId=robotId,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )

        step_counter = 0
        current_force = [0, 0, 0]

        try:
            while True:
                if step_counter % IMPULSE_PERIOD == 0:
                    # Get current end-effector position
                    current_pos = get_tcp_position(gui_client, robotId)
                    
                    print("\nApplying new impulse force...")
                    # Generate random force components
                    current_force = [
                        np.random.uniform(RANDOM_FORCE_MIN, RANDOM_FORCE_MAX),
                        np.random.uniform(RANDOM_FORCE_MIN, RANDOM_FORCE_MAX),
                        np.random.uniform(RANDOM_FORCE_MIN, RANDOM_FORCE_MAX)
                    ]
                    print(f"New force: {current_force}")
                
                # Apply current force every step
                current_pos = get_tcp_position(gui_client, robotId)
                gui_client.applyExternalForce(
                    robotId,
                    linkIndex=num_joints - 1,
                    forceObj=current_force,
                    posObj=current_pos,
                    flags=gui_client.WORLD_FRAME
                )

                for i in range(num_joints):
                    gui_client.setJointMotorControl2(
                        bodyIndex=robotId,
                        jointIndex=i,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=0,
                        positionGain=kp,
                        velocityGain=kv,
                        force=max_force
                    )
                
                base_pos, _ = gui_client.getBasePositionAndOrientation(robotId)
                gui_client.addUserDebugLine(
                    base_pos,
                    current_pos,
                    DEBUG_LINE_COLOR,
                    DEBUG_LINE_WIDTH,
                    lifeTime=DEBUG_LINE_LIFETIME
                )
                
                gui_client.stepSimulation()
                step_counter += 1
                time.sleep(SIMULATION_TIMESTEP)

        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
        
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, terminating workers")
        if pool:
            pool.terminate()
            pool.join()
            
    finally:
        if gui_client:
            print("Disconnecting PyBullet client...")
            gui_client.disconnect()
        
        if pool:
            print("Cleaning up worker processes...")
            pool.close()
            pool.join()