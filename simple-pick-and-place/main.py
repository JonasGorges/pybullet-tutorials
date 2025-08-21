import pybullet as p
import pybullet_data

import util
from util import move_to_joint_pos, gripper_open, gripper_close


def move_to_ee_pose(robot_id, target_ee_pos, target_ee_orientation=None):
    """
    Moves the robot to a given end-effector pose.
    :param robot_id: pyBullet's body id of the robot
    :param target_ee_pos: (3,) list/ndarray with target end-effector position
    :param target_ee_orientation: (4,) list/ndarray with target end-effector orientation as quaternion
    """
    # Calculate inverse kinematics
    if target_ee_orientation is None:
        # If no orientation specified, use current orientation
        current_pos, current_quat, *_ = p.getLinkState(
            robot_id, 
            util.ROBOT_EE_LINK_ID, 
            computeForwardKinematics=True
        )
        target_ee_orientation = current_quat
    
    # Calculate joint positions using inverse kinematics
    joint_pos = p.calculateInverseKinematics(
        robot_id,
        util.ROBOT_EE_LINK_ID,
        targetPosition=target_ee_pos,
        targetOrientation=target_ee_orientation,
        maxNumIterations=100,
        residualThreshold=0.001
    )
    
    # Extract only the first 7 joints (arm joints, ignore gripper joints)
    arm_joint_pos = joint_pos[:7]
    
    # Move the robot to the computed joint configuration
    move_to_joint_pos(robot_id, arm_joint_pos)


def main():
    # connect to pybullet with a graphical user interface
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(1.7, 60, -30, [0.2, 0.2, 0.25])

    # basic configuration
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # allows us to load plane, robots, etc.
    plane_id = p.loadURDF('plane.urdf')  # function returns an ID for the loaded body

    # load the robot
    robot_id = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)

    # load an object to grasp and a box
    object_id = p.loadURDF('cube_small.urdf', basePosition=[0.5, -0.3, 0.025], baseOrientation=[0, 0, 0, 1])
    p.resetVisualShapeData(object_id, -1, rgbaColor=[1, 0, 0, 1])
    tray_id = p.loadURDF('tray/traybox.urdf', basePosition=[0.5, 0.5, 0.0], baseOrientation=[0, 0, 0, 1])

    # Debug: Print object IDs and their information
    print(f"DEBUG: Plane ID: {plane_id}")
    print(f"DEBUG: Robot ID: {robot_id}")
    print(f"DEBUG: Cube (object) ID: {object_id}")
    print(f"DEBUG: Tray ID: {tray_id}")
    
    # List all bodies in the simulation
    num_bodies = p.getNumBodies()
    print(f"DEBUG: Total bodies in simulation: {num_bodies}")
    for i in range(num_bodies):
        info = p.getBodyInfo(i)
        print(f"  Body {i}: {info[0].decode('utf-8') if info[0] else 'No name'}")

    print('******************************')
    input('press enter to start simulation')
    config1 = [-0.7854, 0.75, -1.3562, -1.5708, 0.0, 1.5708, 0.7854]
    config2 = [0.7854, 0.1, -0.7854, -2.1, 0.0, 1.5708, 0.7854]

    # COMMENTED OUT: Forward kinematics demonstration
    # print('going to home configuration')
    # move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)
    # gripper_open(robot_id)
    # 
    # # Show forward kinematics - print end-effector pose
    # pos, quat, *_ = p.getLinkState(robot_id, util.ROBOT_EE_LINK_ID, computeForwardKinematics=True)
    # print(f'Home position - EE pos: {pos}, EE orientation: {quat}')
    # 
    # print('going to configuration 1')
    # move_to_joint_pos(robot_id, config1)
    # pos, quat, *_ = p.getLinkState(robot_id, util.ROBOT_EE_LINK_ID, computeForwardKinematics=True)
    # print(f'Config 1 - EE pos: {pos}, EE orientation: {quat}')
    # 
    # print('going to configuration 2')
    # move_to_joint_pos(robot_id, config2)
    # pos, quat, *_ = p.getLinkState(robot_id, util.ROBOT_EE_LINK_ID, computeForwardKinematics=True)
    # print(f'Config 2 - EE pos: {pos}, EE orientation: {quat}')
    # 
    # print('going to home configuration')
    # move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)

    # print('\n******************************')
    # input('Press enter to test inverse kinematics with move_to_ee_pose...')
    # 
    # # Test moving to specific end-effector positions
    # print('Testing move_to_ee_pose function...')
    # 
    # # Move to a position above the cube
    # cube_pos, _ = p.getBasePositionAndOrientation(object_id)
    # above_cube_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.2]  # 20cm above cube
    # 
    # print(f'Moving to position above cube: {above_cube_pos}')
    # move_to_ee_pose(robot_id, above_cube_pos)
    # 
    # # Move to a position above the tray
    # tray_pos, _ = p.getBasePositionAndOrientation(tray_id)
    # above_tray_pos = [tray_pos[0], tray_pos[1], tray_pos[2] + 0.3]  # 30cm above tray
    # 
    # print(f'Moving to position above tray: {above_tray_pos}')
    # move_to_ee_pose(robot_id, above_tray_pos)

    print('\n******************************')
    print('Starting pick and place sequence directly!')

    # PICK AND PLACE SEQUENCE
    print('Starting pick and place sequence!')
    
    # Step 1: Move to home and open gripper
    print('1. Moving to home position and opening gripper')
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)
    gripper_open(robot_id)
    
    # Step 2: Move to pre-grasp position (above cube)
    print('2. Moving to pre-grasp position above cube')
    cube_pos, _ = p.getBasePositionAndOrientation(object_id)
    pre_grasp_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.1]  # 10cm above cube
    move_to_ee_pose(robot_id, pre_grasp_pos)
    
    # Step 3: Move down to grasp the cube
    print('3. Moving down to grasp cube')
    grasp_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.02]  # Just above cube surface
    move_to_ee_pose(robot_id, grasp_pos)
    
    # Step 4: Close gripper to grasp cube
    print('4. Closing gripper to grasp cube')
    gripper_close(robot_id)
    
    # Step 5: Lift the cube
    print('5. Lifting the cube')
    lift_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.30]  # Lift 15cm
    move_to_ee_pose(robot_id, lift_pos)
    
    # Step 6: Move to position above tray
    print('6. Moving to position above tray')
    tray_pos, _ = p.getBasePositionAndOrientation(tray_id)
    above_tray_pos = [tray_pos[0], tray_pos[1], tray_pos[2] + 0.30]  # 15cm above tray
    move_to_ee_pose(robot_id, above_tray_pos)
    
    # Step 7: Lower to drop position
    print('7. Lowering to drop position')
    drop_pos = [tray_pos[0], tray_pos[1], tray_pos[2] + 0.08]  # 8cm above tray
    move_to_ee_pose(robot_id, drop_pos)
    
    # Step 8: Open gripper to release cube
    print('8. Opening gripper to release cube')
    gripper_open(robot_id)
    
    # Step 9: Return to home
    print('9. Returning to home position')
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)
    
    print('Pick and place sequence completed! 🎉')

    print('program finished. hit enter to close.')
    input()
    # clean up
    p.disconnect()


if __name__ == '__main__':
    main()
