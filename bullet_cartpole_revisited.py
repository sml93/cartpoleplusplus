#!/usr/bin/env python

from collections import *
import gym
import sys
import time
import numpy as np
import pybullet as p

from gym import spaces

np.set_printoptions(precision=3, suppress=True, linewidth=10000)


def add_opts(parser):
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--delay', type=float, default=0.0)
    parser.add_argument('--action-force', type=float, default=50.0,
                        help="magnitude of action force applied per step")
    parser.add_argument('--initial-force', type=float, default=55.0,
                        help="magnitude of initial push, in random direction")
    parser.add_argument('--no-random-theta', action='store_true')
    parser.add_argument('--action-repeats', type=int, default=2,
                        help="number of action repeats")
    parser.add_argument('--steps-per-repeat', type=int, default=5,
                        help="number of sim steps per repeat")
    parser.add_argument('--num-cameras', type=int, default=1,
                        help="how many camera points to render; 1 or 2")
    parser.add_argument('--event-log-out', type=str, default=None,
                        help="path to record event log.")
    parser.add_argument('--max-episode-len', type=int, default=200,
                        help="maximum episode len for cartpole")
    parser.add_argument('--use-raw-pixels', action='store_true',
                        help="use raw pixels as state instead of cart/pole poses")
    parser.add_argument('--render-width', type=int, default=50,
                        help="if --use-raw-pixels render with this width")
    parser.add_argument('--render-height', type=int, default=50,
                        help="if --use-raw-pixels render with this height")
    parser.add_argument('--reward-calc', type=str, default='fixed',
                        help="'fixed': 1 per step. 'angle': 2*max_angle - ox - oy. 'action': 1.5 - |action|."
                             "'angle_action': both angle and action")


def state_fields_of_pose_of(body_id):
    (x, y, z), (a, b, c, d) = p.getBasePositionAndOrientation(body_id)
    return np.array([x, y, z, a, b, c, d])


class UAV:
    """ class for UAVwFluid """
    def __init__(self, uav, client):
        # Joint indx as found by p.getJointInfo()
        self.uav = None
        self.ground = None
        self.joint = 0
        self.joint_spd = 0
        self.c_drag = 0.01
        self.angle = 0.
        self.client = client
        self.uav = uav

    def get_ids(self):
        return self.uav, self.client

    def apply_angle(self, ang):
        """ Expects action to be 90 - tilt angle of the fluid (beta) """
        # ang -= (pi/2)
        # self.angle = max(min(ang, -0.349), 0.785)
        self.angle = ang - (np.pi/4)
        p.setJointMotorControl2(self.uav, self.joint,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self.angle)

    def get_observation(self):
        (x, y, z), orient = p.getBasePositionAndOrientation(self.uav, self.client)
        ox, oy, oz = p.getEulerFromQuaternion(orient)  # Row / Pitch / Yaw
        return x, y, z, ox, oy, oz


class BulletCartpole(gym.Env):
    def __init__(self, opts, discrete_actions):
        self.gui = opts.gui
        self.delay = opts.delay if self.gui else 0.0
        self.client = p.connect(p.GUI)

        self.max_episode_len = opts.max_episode_len

        ''''
        Threshold for pole position 
        If absolute x or y moves outof this, end episode.
        '''
        self.pos_threshold = 2.0

        '''
        Threshold for angle from z-axis. If x or y > this, end episode.
        '''
        self.angle_threshold = 0.4  # radians (approx. 11.46 degrees)

        '''
        Force to apply per action simulation step.
        In the discrete case, this is the fixed force applied
        In the continuous case, each x/y is in range(-F, F)
        '''
        self.action_force = opts.action_force

        '''
        Initial push force. No action will always result in pole falling after intial_force_steps
        But no so much to the point that it cannot be recovered. Also see initial_force_steps.
        '''
        self.initial_force = opts.initial_force

        '''
        Number of sim steps initial force is applied (see intial_force).
        '''
        self.initial_force_steps = 30

        '''
        Whether doing an initial push in a random direction or not.
        If false, always push along x-axis (simple problem, useful for debugging)
        '''
        self.random_theta = not opts.no_random_theta

        '''
        True, if action space is discrete; 5 values: do nothing, left, right, up & down
        False, if action space is continuous; fx, fy both: (-action_force, action_force)
        '''
        self.discrete_actions = discrete_actions

        '''
        5 discrete actions: do nothing, left, right, up, down
        2 continuous action elements: fx & fy
        '''
        if self.discrete_actions:
            self.action_space = spaces.Discrete(5)
        else:
            self.action_space = spaces.Box(-1.0, 1.0, shape=(1, 2))

        ''' Open event log '''
        if opts.event_log_out:
            import event_log
            self.event_log = event_log.EventLog(opts.event_log_out, opts.use_raw_pixels)
        else:
            self.event_log = None

        '''
        How many time to repeat each action per step().
        And how many sim steps to do per state capture
        (Total number of sim steps = action_repeats * steps_per_repeat
        '''
        self.repeats = opts.action_repeats
        self.steps_per_repeat = opts.steps_per_repeat

        '''
        How many cameras to render?
        If 1, just render from the front
        If 2, render from front and 90 deg side
        '''
        if opts.num_cameras not in [1, 2]:
            raise ValueError("--num-cameras must be 1 or 2")
        self.num_cameras = opts.num_cameras

        ''' Whether using raw pixels for state or just pole + cart pose '''
        self.use_raw_pixels = opts.use_raw_pixels

        ''' If the use_raw_pixels is set, render '''
        self.render_width = opts.render_width
        self.render_height = opts.render_height

        ''' Decide observation space '''
        if self.use_raw_pixels:
            '''
            In high dimensional cases, each observation is an RGB image: (H, W, 3-channels)
            We have R repeats and C cameras resulting in (H, W, 3-channels, C, R)
            Final state fed to network is concatenated in depth => (H, W, 3*C*R)
            '''
            state_shape = (self.render_height, self.render_width, 3, self.num_cameras, self.repeats)
        else:
            '''
            In the low dim cases, obs space from problem is (R, 2, 7) where,
            R = number of repeats,
            2 = Two items, cart & pole
            7d tuple for position + orientation pose
            '''
            state_shape = (self.repeats, 2, 7)
        float_max = np.finfo(np.float32).max
        self.observation_space = gym.spaces.Box(-float_max, float_max, state_shape)

        ''' Check reward type '''
        assert opts.reward_calc in ['fixed', 'angle', 'action', 'angle_action']
        self.reward_calc = opts.reward_calc

        ''' No state until reset '''
        self.state = np.empty(state_shape, dtype=np.float32)

        ''' Setup bullet '''
        p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        '''
        Amending the first set of tuple will tell it where to place the object,
        second tuple is the orientation of it
        '''
        p.loadURDF("models/ground.urdf", 0,0,0, 0,0,0,1)
        self.cart = p.loadURDF("models/poc.urdf", 0, 0, 0.1,
                               0, 0, 0, 1)  # To change to a fluid and its params, change inside the urdf
        self.pole = p.loadURDF("models/uav_fluid.urdf", 0, 0, 0.15,
                               0, 0, 0, 1)  # To change to a UAV and its params, change inside the urdf
        #self.car = p.loadURDF("models/simplecar.urdf", 1, 1, 0.12,
        #                       0, 0, 0, 1)
        # self.uav = p.loadURDF("models/cart.urdf", 0, 0, 0.9,
        #                       0, 0, 0, 1)  # Test UAV
        # self.cart = p.loadURDF("models/cart.urdf", 0,0,0.35, 0,0,0,1)
        # self.pole = p.loadURDF("models/pole.urdf", 0,0,0.08, 0,0,0,1)

    def configure(self, display=None):
        pass

    def seed(self, seed=None):
        pass

    def render(self, mode, close):
        pass

    def step(self, action):
        """ Check if the simulation is done """
        if self.done:
            print >> sys.stderr, "Calling step after done??"
            return np.copy(self.state), 0, True, {}

        info = {}

        ''' Based on action, decide the x and y forces '''
        fx = fy = 0
        if self.discrete_actions:
            if action == 0:
                pass
            elif action == 1:
                fx = self.action_force
            elif action == 2:
                fx = -self.action_force
            elif action == 3:
                fy = self.action_force
            elif action == 4:
                fy = -self.action_force
            else:
                raise Exception("Unknown discrete action [%s]" % action)
        else:
            fx, fy = action[0] * self.action_force

        '''
        Step simulation forward.
        At the end of each repeat, set part of the step's state
        by capturing the cart & pole state in some form.
        '''
        for r in range(self.repeats):
            for _ in range(self.steps_per_repeat):
                p.stepSimulation()
                p.applyExternalForce(self.cart, -1, (fx, fy, 0), (0, 0, 0), p.WORLD_FRAME)
                if self.delay > 0:
                    time.sleep(self.delay)
            self.set_state_element_for_repeat(r)
        self.steps += 1

        '''
        Check for out of bounds by position or orientation on pole
        Fetch pose explicitly rather than depending on fields in state
        '''
        move = UAV(self.pole, self.client)
        x, y, _z, ox, oy, _oz = move.get_observation()
        #print (x, y, ox, oy)
        move.apply_angle(oy)
        (x, y, _z), orient = p.getBasePositionAndOrientation(self.pole)
        ox, oy, _oz = p.getEulerFromQuaternion(orient)  # Roll / Pitch / Yaw
        if abs(x) > self.pos_threshold or abs(y) > self.pos_threshold:
            info['done_reason'] = 'out of position bounds'
            self.done = True
        elif abs(ox) > self.angle_threshold or abs(oy) > self.angle_threshold:
            info['done_reason'] = 'out of orientation bounds'
            self.done = True

        ''' Calculate reward, fixed base of 1.0 '''
        reward = 1.0
        if self.reward_calc == "angle" or self.reward_calc == "angle_action":
            # Clip to zero since angles can be past threshold
            reward += max(0, 2 * self.angle_threshold - np.abs(ox) - np.abs(oy))
        if self.reward_calc == "action" or self.reward_calc == "angle_action":
            '''
            Max norm will be sqr(2) ~= 1.4
            Reward is already 1.0 to add another 0.5 as o0.1 buffer from zero
            '''
            reward += 0.5 - np.linalg.norm(action[0])

        ''' Log this event '''
        if self.event_log:
            self.event_log.add(self.state, action, reward)

        # Return observation
        return np.copy(self.state), reward, self.done, info

    def render_rgb(self, camera_idx):
        cameraPos = [(0.0, 0.75, 0.75), (0.75, 0.0, 0.75)][camera_idx]
        targetPos = (0, 0, 0.3)
        cameraUp = (0, 0, 1)
        nearVal, farVal = 1, 20
        fov = 60
        _w, _h, rgba, _depth, _objects = p.renderImage(self.render_width, self.render_height,
                                                       cameraPos, targetPos, cameraUp,
                                                       nearVal, farVal, fov)
        '''
        Convert from 1d uint8 to array to (H,W,3) hacky hardcode whitened float16 array.
        And normalise 0 -> 1 whiten later
        '''
        rgba_img = np.reshape(np.asarray(rgba, dtype=np.float16),
                              (self.render_height, self.render_width, 4))
        rgb_img = rgba_img[:, :, :3]
        rgb_img /= 255
        return rgb_img

    def set_state_element_for_repeat(self, repeat):
        if self.use_raw_pixels:
            '''
            High dim cases (H, W, 3, C, R)
            H, W, 3 -> height x width, 3 channel RGB image
            C -> camera idx: 0 or 1
            R -> repeat
            '''
            for camera_idx in range(self.num_cameras):
                self.state[:, :, :, camera_idx, repeat] = self.render_rgb(camera_idx)
        else:
            '''
            Low dim case states is (R, 2, 7)
            R -> repeat, 2 -> 2 object (cart, pole), 7 -> 7d pose
            '''
            self.state[repeat][0] = state_fields_of_pose_of(self.cart)
            self.state[repeat][1] = state_fields_of_pose_of(self.pole)

    def reset(self):
        '''
        Reset state
        '''
        self.steps = 0
        self.done = False

        '''
        Reset pole on cart in starting poses
        '''
        p.resetBasePositionAndOrientation(self.cart, (0, 0, 0.1), (0, 0, 0, 1))  # 2nd tuple is orientation
        p.resetBasePositionAndOrientation(self.pole, (0, 0, 0.15), (0, 0, 0, 1))
        # p.resetBasePositionAndOrientation(self.car, (1, 1, 0.12), (0, 0, 0, 1))
        # p.resetBasePositionAndOrientation(self.uav, (0, 0, 0.9), (0, 0, 0, 1))

        for _ in range(100):
            p.stepSimulation()

        '''
        Give a fixed force push in a random direction to get things going
        '''
        theta = (np.random.random() * 2 * np.pi) if self.random_theta else 0.0
        fx, fy = self.initial_force * np.cos(theta), self.initial_force * np.sin(theta)

        for _ in range(self.initial_force_steps):
            p.stepSimulation()
            ''' this is the part where the force is applied onto, can try to change onto the pole '''
            p.applyExternalForce(self.cart, -1, (fx, fy, 0), (0, 0, 0), p.WORLD_FRAME)
            if self.delay > 0:
                time.sleep(self.delay)

        '''
        Bootstrap state by running for all repeats
        '''
        for i in range(self.repeats):
            self.set_state_element_for_repeat(i)

        '''
        Reset event log (if applicable) and add entry with only state 
        '''
        if self.event_log:
            self.event_log.reset()
            self.event_log.add_just_state(self.state)

        '''
        Return this state
        '''
        return np.copy(self.state)
