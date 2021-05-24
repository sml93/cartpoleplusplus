#!/usr/bin/env python
import os
import pybullet as p


class POC:
    def __init__(self, client, base):
        f_name = os.path.join(os.path.dirname(__file__), 'poc.urdf')
        p.loadURDF(fileName=f_name,
                   basePosition=[0, 0, 0],
                   physicsClientId=client)
