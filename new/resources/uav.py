#!/usr/bin/env python
import os
import numpy as np
import pybullet as p


class UAV:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'uav_fluid.urdf')
        self.uav = p.loadURDF(fileName=f_name,
                              basePosition=[0, 0, 0.15],
                              baseOrientation=[0, 0, 0, 1],
                              physicsClientId=client)
        # self.numJoint = p.getNumJoints(self.uav)
        # for joint_num in range(self.numJoint):
        #     self.joint = p.getJointInfo(self.uav, joint_num)
        # print(self.joint[0])
        self.angle = 0

    def get_ids(self):
        return self.uav, self.client

    def apply_angle(self, ang, jointIdx):
        # self.joint = jointIdx
        # self.angle = max(min(ang, -0.349), 0.524)
        self.angle = ang
        p.setJointMotorControl2(self.uav, jointIdx,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.angle,
                                physicsClientId=self.client)

    def get_obs(self):
        (x, y, z), orient = p.getBasePositionAndOrientation(self.uav)
        ox, oy, oz = p.getEulerFromQuaternion(orient)
        return x, y, z, ox, oy, oz


# def main():
#     client = p.connect(p.GUI)
#     uav = UAV(client)
#     x, y, _z, ox, oy, _oz = uav.get_obs()
#     print(x, y, _z, ox, oy, _oz)
#     angle = p.addUserDebugParameter('Angle', -0.4, 0.55)
#     joint = [0]
#     print(joint)
#     while True:
#         user_angle = p.readUserDebugParameter(angle)
#         # print(user_angle)
#         for joint_index in joint:
#             print(joint_index)
#             uav.apply_angle(user_angle, joint_index)
#         p.stepSimulation()
#
#
# if __name__ == "__main__":
#     client = p.connect(p.GUI)
#     uav = UAV(client)
#     x, y, _z, ox, oy, _oz = uav.get_obs()
#     print(x, y, _z, ox, oy, _oz)
#     angle = p.addUserDebugParameter('Angle', -0.4, 0.55)
#     joint = [0]
#     print(joint)
#     while True:
#         user_angle = p.readUserDebugParameter(angle)
#         # print(user_angle)
#         for joint_index in joint:
#             print(joint_index)
#             uav.apply_angle(user_angle, joint_index)
#         p.stepSimulation()
