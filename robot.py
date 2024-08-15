import pybullet as p
import numpy as np

class Robot:
    def __init__(self, client):
        self.client = client
        self.robot_id, self.joints = self.load_robot()

    def load_robot(self):
        robot_id = p.loadURDF(
            "urdf/ur10_robot.urdf",
            [0, 0, 0],
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,
        )
        joints = []
        for joint_id in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, joint_id)
            if info[2] != p.JOINT_FIXED:
                joints.append({
                    "jointID": info[0],
                    "jointName": info[1].decode("utf-8"),
                    "jointMaxForce": info[10],
                    "jointMaxVelocity": info[11],
                })
        return robot_id, joints

    def reset(self):
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])

    def get_end_effector_position(self):
        print(f'robot_id: {self.robot_id}')
        print(f'Joints : {self.joints}')
        print(f'Num of Joints: {p.getNumJoints(self.robot_id)}')
        return np.array(p.getLinkState(self.robot_id, linkIndex=10, computeLinkVelocity=False)[0])

    def get_joint_angles(self):
        joint_angles = []
        for joint in self.joints:
            joint_info = p.getJointState(self.robot_id, joint["jointID"])
            joint_angles.append(joint_info[0])
        return np.array(joint_angles)

    def move(self, target_position, target_orientation):
        joint_poses = p.calculateInverseKinematics(self.robot_id, 10, target_position, targetOrientation=target_orientation)
        for i, joint in enumerate(self.joints):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint["jointID"],
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=joint.get("jointMaxForce", 500),
                maxVelocity=joint.get("jointMaxVelocity", 1.0)
            )
        p.stepSimulation()
