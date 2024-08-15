import pybullet as p
import pybullet_data as pd
import numpy as np
import time

from utils.utils import euclidean_distance
from utils.pb_utils import create_marker_point


def load_robot():
    """
    Loads a UR10 robot model in the PyBullet physics simulation environment.
    """
    robot_id = p.loadURDF(
        "urdf/ur10_robot.urdf",
        [0, 0, 0],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True,
    )
    joint_type = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
    joints = []

    for joint_id in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_id)
        data = {
            "jointID": info[0],
            "jointName": info[1].decode("utf-8"),
            "jointType": joint_type[info[2]],
            "jointLowerLimit": info[8],
            "jointUpperLimit": info[9],
            "jointMaxForce": info[10],
            "jointMaxVelocity": info[11],
        }
        if data["jointType"] != "FIXED":
            joints.append(data)
    return robot_id, joints

def get_arm_position(robot_id):
    """
    Get the position of the arm's end effector.
    """
    return np.array(p.getLinkState(robot_id, linkIndex=10, computeLinkVelocity=False)[0])

def move_robot(robot_id, joints, target_position):
    """
    Move the robot to the target position.
    """
    old_position = get_arm_position(robot_id)
    old_angles = get_joint_angles(robot_id)

    time.sleep(1)

    end_effector_index = 10  # Update this if necessary
    joint_poses = p.calculateInverseKinematics(robot_id, end_effector_index, target_position)

    for i, joint in enumerate(joints):
        joint_index = joint["jointID"]
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[i],
            force=joint.get("jointMaxForce", 500),
            maxVelocity=joint.get("jointMaxVelocity", 1.0)
        )

    while True:
        current_position = get_arm_position(robot_id)
        if euclidean_distance(current_position, target_position) < 0.01:
            break

        p.stepSimulation()
        time.sleep(1. / 240)

    new_position = get_arm_position(robot_id)
    new_angles = get_joint_angles(robot_id)
    
    print(f'Old Arm Position : {old_position}')
    print(f'New Arm Position : {new_position}')
    print(f'Old Joint Angles : {old_angles}')
    print(f'New Joint Angles : {new_angles}')

    time.sleep(1)

def get_joint_angles(robot_id):
    """
    Get the joint angles of the robot.
    """
    joint_angles = []
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointState(robot_id, i)
        joint_angles.append(joint_info[0])
    return joint_angles

def pick_and_place(robot_id, joints, pick_position, place_position):
    """
    Perform the pick and place operation.

    Parameters:
    - robot_id: The ID of the robot.
    - joints: The list of joints of the robot.
    - pick_position: The position to pick the object from.
    - place_position: The position to place the object.
    """

    # Move above the pick position
    above_pick_position = pick_position.copy()
    above_pick_position[2] += 0.1  # move slightly above the object
    move_robot(robot_id, joints, above_pick_position)

    # Move to the pick position
    move_robot(robot_id, joints, pick_position)

    # Simulate picking up the object (e.g., closing the gripper)
    # This would involve controlling the gripper, but we'll assume the object is picked

    # Move back above the pick position
    move_robot(robot_id, joints, above_pick_position)

    # Move above the place position
    above_place_position = place_position.copy()
    above_place_position[2] += 0.1  # move slightly above the place position
    move_robot(robot_id, joints, above_place_position)

    # Move to the place position
    move_robot(robot_id, joints, place_position)

    # Simulate placing the object (e.g., opening the gripper)
    # Again, this would involve controlling the gripper, but we'll assume the object is placed

    # Move back above the place position
    move_robot(robot_id, joints, above_place_position)


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    robot_id, joints = load_robot()

    # Create pick and place markers
    pick_position = [0.5, 0.0, 0.1]
    place_position = [0.0, 0.5, 0.1]

    create_marker_point(*pick_position, size=[0.02, 0.02, 0.02], colour=[1, 0, 0, 1])  # Red marker
    create_marker_point(*place_position, size=[0.02, 0.02, 0.02], colour=[0, 1, 0, 1])  # Green marker

    # Perform pick and place operation
    pick_and_place(robot_id, joints, pick_position, place_position)

    # Disconnect from the simulation
    p.disconnect()
