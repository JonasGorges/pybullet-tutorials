import time

import pybullet as p
import pybullet_data
import numpy as np

import util


def simulate(steps=None, seconds=None, slow_down=True):
    """
    Wraps pybullet's stepSimulation function and allows some more control over duration.
    Will simulate for a number of steps, or number of seconds, whichever is reached first.
    If both are None, it will run indefinitely.

    :param steps: int, number of steps to simulate
    :param seconds: float, number of seconds to simulate
    :param slow_down: bool, if set to True will slow down the simulated time to be aligned to real time
    """
    dt = 1./240  # a single timestep is 1/240 seconds per default
    seconds_passed = 0.0
    steps_passed = 0
    start_time = time.time()

    while True:
        p.stepSimulation()
        steps_passed += 1
        seconds_passed += dt

        if slow_down:
            time_elapsed = time.time() - start_time
            wait_time = seconds_passed - time_elapsed
            time.sleep(max(wait_time, 0))
        if steps is not None and steps_passed > steps:
            break
        if seconds is not None and seconds_passed > seconds:
            break


def move_to_joint_pos(robot_id, target_joint_pos, max_velocity=1):
    """
    Move robot to target joint configuration using position control.
    
    :param robot_id: int, body id of the robot
    :param target_joint_pos: list, target joint positions for all 7 joints
    :param max_velocity: float, maximum velocity for each joint
    """
    # Set position control for all joints
    for joint_id in range(7):
        p.setJointMotorControl2(
            robot_id,
            joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_joint_pos[joint_id],
            maxVelocity=max_velocity,
            force=100
        )
    
    # Simulate until robot reaches target position
    while True:
        simulate(steps=1, slow_down=False)
        current_joint_pos = util.get_arm_joint_pos(robot_id)
        
        # Check if we've reached the target (within tolerance)
        if np.allclose(current_joint_pos, target_joint_pos, atol=0.0001):
            print(f"Reached target position: {target_joint_pos}")
            break


def move_to_joint_pos_synchronized(robot_id, target_joint_pos, max_velocity=1):
    """
    Move robot to target joint configuration using synchronized PTP.
    All joints will arrive at their targets at the same time for smoother motion.
    
    :param robot_id: int, body id of the robot
    :param target_joint_pos: list, target joint positions for all 7 joints
    :param max_velocity: float, maximum velocity for the joint that travels furthest
    """
    # Get current joint positions
    current_joint_pos = np.array(util.get_arm_joint_pos(robot_id))
    target_joint_pos = np.array(target_joint_pos)
    
    # Calculate distances each joint needs to travel
    distances = np.abs(target_joint_pos - current_joint_pos)
    
    # Find the maximum distance (this joint will use max_velocity)
    max_dist = np.max(distances)
    
    # Avoid division by zero
    if max_dist == 0:
        print("Already at target position!")
        return
    
    # Calculate velocity for each joint (synchronized)
    joint_velocities = []
    for i in range(7):
        # Scale velocity based on distance ratio
        joint_vel = (distances[i] / max_dist) * max_velocity
        joint_velocities.append(max(joint_vel, 0.01))  # Minimum velocity to avoid issues
    
    print(f"Synchronized velocities: {[f'{v:.3f}' for v in joint_velocities]}")
    
    # Set position control for all joints with individual velocities
    for joint_id in range(7):
        p.setJointMotorControl2(
            robot_id,
            joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_joint_pos[joint_id],
            maxVelocity=joint_velocities[joint_id],
            force=100
        )
    
    # Simulate until robot reaches target position
    while True:
        simulate(steps=1, slow_down=False)
        current_joint_pos = util.get_arm_joint_pos(robot_id)
        
        # Check if we've reached the target (within tolerance)
        if np.allclose(current_joint_pos, target_joint_pos, atol=0.0001):
            print(f"Reached target position (synchronized): {target_joint_pos}")
            break


def main():
    # connect to pybullet with a graphical user interface
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # basic configuration
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # allows us to load plane, robots, etc.
    plane_id = p.loadURDF('plane.urdf')  # function returns an ID for the loaded body

    # load a robot
    robot_id = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)

    # Reset robot to home position
    for i in range(7):
        p.resetJointState(robot_id, i, util.ROBOT_HOME_CONFIG[i])
    
    print("Using velocity control to move joint 0 to position 1...")
    
    joint_id = 0

    # Set velocity control command once (per tutorial)
    p.setJointMotorControl2(
        robot_id, 
        joint_id, 
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=1,
        force=100
    )

    # Get current joint position
    joint_position = p.getJointState(robot_id, joint_id)[0]

    # Step simulation one step at a time while joint position < 1
    while joint_position < 1.0:
        simulate(steps=1, slow_down=False)  # simulate one step
        joint_position = p.getJointState(robot_id, joint_id)[0]
        print(f"Joint {joint_id} position: {joint_position:.4f}")

    # Stop the joint once we reach the position
    p.setJointMotorControl2(
        robot_id, 
        joint_id, 
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=100
    )
    
    final_position_velocity = p.getJointState(robot_id, joint_id)[0]
    print(f"Velocity control - Target: 1.0, Final: {final_position_velocity:.4f}, Error: {abs(1.0 - final_position_velocity):.4f}")
    
    print("\n" + "="*50)
    print("Now using POSITION control to move joint 0 to position 2...")
    
    # Using position control to move to target position of 2
    p.setJointMotorControl2(
        robot_id, 
        joint_id, 
        controlMode=p.POSITION_CONTROL,
        targetPosition=2,
        maxVelocity=0.2,  # Much slower for visible movement
        force=100
    )
    
    # Position control automatically handles the movement, just simulate
    simulate(seconds=8)  # More time to see the movement
    
    final_position_position = p.getJointState(robot_id, joint_id)[0]
    print(f"Position control - Target: 2.0, Final: {final_position_position:.4f}, Error: {abs(2.0 - final_position_position):.4f}")
    
    print("\n" + "="*50)
    input("Press Enter to continue to PTP motion demonstrations...")
    
    print("Demonstrating REGULAR PTP motion...")
    
    # Define three different robot configurations
    config1 = [0.5, -0.5, 0.5, -2.0, 0.5, 1.0, 0.5]
    config2 = [-0.5, -1.0, -0.5, -1.5, -0.5, 2.0, -0.5]  
    config3 = util.ROBOT_HOME_CONFIG  # Back to home
    
    # Move through the configurations using regular PTP
    configurations = [config1, config2, config3]
    
    for i, config in enumerate(configurations, 1):
        print(f"\nRegular PTP - Moving to configuration {i}: {config}")
        move_to_joint_pos(robot_id, config, max_velocity=0.3)  # Slower movement
        simulate(seconds=2, slow_down=True)  # Longer pause to see the result
    
    print("\n" + "="*60)
    input("Press Enter to see SYNCHRONIZED PTP motion...")
    
    print("Demonstrating SYNCHRONIZED PTP motion...")
    print("Notice how all joints arrive at the same time for smoother motion!")
    
    # Reset to home and demonstrate synchronized PTP
    print(f"\nSynchronized PTP - Moving to home position")
    move_to_joint_pos_synchronized(robot_id, util.ROBOT_HOME_CONFIG, max_velocity=0.3)  # Slower movement
    simulate(seconds=2, slow_down=True)
    
    # Move through the same configurations using synchronized PTP
    for i, config in enumerate(configurations, 1):
        print(f"\nSynchronized PTP - Moving to configuration {i}: {config}")
        move_to_joint_pos_synchronized(robot_id, config, max_velocity=0.3)  # Slower movement
        simulate(seconds=2, slow_down=True)  # Longer pause to see the result

    # clean up
    p.disconnect()
    print('program finished. bye.')


if __name__ == '__main__':
    main()
