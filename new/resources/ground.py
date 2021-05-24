#!/usr/bin/env python
import os
import pybullet as p


class Ground:
    def __init__(self, client):
        f_name = os.path.join(os.path.dirname(__file__), 'ground.urdf')
        p.loadURDF(fileName=f_name,
                   basePosition=[0, 0, 0],
                   physicsClientId=client)
