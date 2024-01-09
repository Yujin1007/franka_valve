#!/usr/bin/env python3
import sys
sys.path.append('/home/kist-robot2/catkin_ws/src/franka_overall')
import numpy as np
import sys
from numpy.linalg import inv
# sys.path.append('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/build/devel/lib/franka_emika_panda')
# from build.devel.lib import controller
from build import controller


# from build.devel.lib.controller_CLKI_A import controller
import mujoco
import gym
from gym import spaces
from random import random, randint, uniform
from scipy.spatial.transform import Rotation as R
from mujoco import viewer
from time import sleep
import tools
import rotations
import torch
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
import matplotlib.cm as cm


BODY = 1
JOINT = 3
GEOM = 5
MOTION_TIME_CONST = 10.
TASK_SPACE_TIME = 3+1+0.5

RL = 2
MANUAL = 1

RPY = False
XYZRPY = True

JOINT_CONTROL = 1
TASK_CONTROL = 2
CIRCULAR_CONTROL = 3
RL_CIRCULAR_CONTROL = 4
RL_CONTROL = 6

def BringClassifier(path):
    # input_size = 8
    # output_size = 36
    # classifier = Classifier(input_size, output_size)
    # device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    # classifier.to(device)
    classifier = torch.load(path)
    classifier.eval()
    return classifier




class fr3_reset:
    metadata = {"render_modes": ["human"], "render_fps": 30}
    def __init__(self) -> None:
        self.k = 7  # for jacobian calculation
        self.dof = 9  # all joints (include gripper joint)
        self.model_path = "/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/scene_valve.xml"
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.controller = controller.CController(self.k)
        self._torque = np.zeros(self.dof, dtype=np.float64)
        self.stack = 5
        self.rendering = False

        self.observation_space = self._construct_observation_space()
        self.action_space = self._construct_action_space()
        ## reward weight
        self.reward_range = None
        self.rw_acc = 3  # np.exp(-sum(abs(action - self.action_pre)))
        self.rw_c = 10  # contact done -> -1
        self.rw_b = 1  # joint boundary done -> -1
        self.rw_gr = 1  # 1/-1 grasp


        self.viewer = None
        self.env_rand = False
        self.train = True
        self.q_range = self.model.jnt_range[:self.k]
        self.qdot_range = np.array([[-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750],
                                    [-2.61, 2.61], [-2.61, 2.61], [-2.61, 2.61]])
        self.qdot_init = [0,0,0,0,0,0,0,0,0,0,0]
        self.q_init = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45),0,0,0,0]
        self.q_reset = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45),0.03,0.03,0,0]
        self.episode_number = -1

        self.classifier_clk = BringClassifier(
            "/home/kist-robot2/catkin_ws/src/franka_emika_panda/py_src/classifier/model_clk.pt")
        self.classifier_cclk = BringClassifier(
            "/home/kist-robot2/catkin_ws/src/franka_emika_panda/py_src/classifier/model_cclk.pt")
        desired_contact_list = ["finger_contact0", "finger_contact1",
                                "finger_contact2", "finger_contact3", "finger_contact4", "finger_contact5",
                                "finger_contact6", "finger_contact7",
                                "finger_contact8", "finger_contact9", "finger0_contact", "finger1_contact",
                                "valve_contact0", "valve_contact1",
                                "handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                "handle_contact4", "handle_contact5", "handle_contact6", "handle_contact7",
                                "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                "handle_contact15","handle_contact16", "handle_contact18", "handle_contact19",
                                "handle_contact21", "handle_contact22", "handle_contact23"]
        desired_contact_list_finger = ["finger_contact1",
                                       "finger_contact2", "finger_contact3", "finger_contact4",
                                       "finger_contact6", "finger_contact7",
                                       "finger_contact8", "finger_contact9", ]
        desired_contact_list_obj = ["handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                    "handle_contact5", "handle_contact6",
                                    "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                    "handle_contact15", "handle_contact16",
                                    "handle_contact21", "handle_contact22", "handle_contact23", "valve_contact0"]

        self.desired_contact_bid = tools.name2id(self.model, GEOM, desired_contact_list)
        self.desired_contact_finger_bid = tools.name2id(self.model, GEOM, desired_contact_list_finger)
        self.desired_contact_obj_bid = tools.name2id(self.model, GEOM, desired_contact_list_obj)

        self.direction_state = ["clk","cclk"]

    def reset(self, direction=None):
        self.control_mode = 0

        self.direction = direction
        cnt_frame = 0
        env_reset = True
        self.episode_time_elapsed = 0.0
        self.handle_angle = 0.0
        self.handle_angle_reset = 0.0
        self.action_reset = 0
        self.cnt_reset = 0
        while env_reset:
            self.episode_number += 1
            self.start_time = self.data.time + 1
            self.controller.initialize()
            self.data.qpos = self.q_init
            self.data.qvel = self.qdot_init

            if self.direction is None:
                self.direction = self.direction_state[randint(0, 1)]

            r, obj, radius, init_angle = self.env_randomization() #self.obs_object initialize

            self.init_angle = init_angle

            if self.direction == "clk":
                self.goal_angle = init_angle - 5* np.pi
            elif self.direction == "cclk":
                self.goal_angle = init_angle + 5 * np.pi
            self.required_angle = abs(self.goal_angle - self.init_angle)

            self.episode_time = abs( MOTION_TIME_CONST * abs(self.goal_angle-self.init_angle) * radius)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
            self.controller.randomize_env(r, obj, self.data.xpos[:22].reshape(66, ), self.init_angle, self.goal_angle, RL, RPY)

            self.controller.control_mujoco()


            self.contact_done = False
            self.bound_done = False
            self.goal_done = False
            self.reset_done = False
            self.action_pre  = np.zeros(6)
            self.drpy_pre  = np.zeros(3)

            self.obs_q = np.zeros([self.stack, self.k])
            self.obs_rpy = np.zeros([self.stack,6])
            self.obs_xyz = np.zeros([self.stack, 3])

            self.rpyfromvalve_data = []
            self.path_data = []

            while self.control_mode != 4:
                self.control_mode = self.controller.control_mode()
                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
                self.controller.control_mujoco()
                self._torque, self.max_rotation = self.controller.write()
                for i in range(self.dof - 1):
                    self.data.ctrl[i] = self._torque[i]

                mujoco.mj_step(self.model, self.data)
                ee = self.controller.get_ee()

                self.path_data.append([ee[1] + self.data.qpos[:7].tolist()])
                done = self._done()
                normalized_q = self.obs_q[0]
                if max(abs(normalized_q)) > 0.95:
                    done = True

                if done:
                    # np.save("path_data.npy", self.path_data)
                    break
                # --- collect observation for initialization ---
                if cnt_frame == 100:
                    cnt_frame = 0
                    end_effector = self.controller.get_ee()
                    # self.save_frame_data(end_effector)
                    obs = self._observation(end_effector)
                cnt_frame += 1
                if self.rendering:
                    self.render()
            if self.control_mode == 4:

                env_reset = False
                self.start_time = self.data.time
                self.q_reset[:self.k] = self.data.qpos[:self.k]

        return obs

    def step(self, action_rotation):
        drpy = tools.orientation_6d_to_euler(action_rotation)
        done = False
        duration = 0
        normalized_q = self.obs_q[0]
        if max(abs(normalized_q)) > 0.95:
            self.action_reset = 1
            self.cnt_reset += 1
            # print(self.cnt_reset, end="|")
            # if self.cnt_reset >= 10:
            #     self.rendering = True

        else:
            self.action_reset = 0

        if self.action_reset:
            self.handle_angle_reset += max(abs(self.data.qpos[-2:]))
            self.data.qpos= self.q_reset
            self.data.qvel = self.qdot_init
            mujoco.mj_step(self.model, self.data)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
            self.controller.target_replan()
            if self.rendering:
                    self.render()
        else:
            while not done:
                done = self._done()
                self.control_mode = self.controller.control_mode()
                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))

                # --- RL controller input ---
                if self.control_mode == RL_CIRCULAR_CONTROL:
                    drpy_tmp = (drpy - self.drpy_pre) / 100 * duration + self.drpy_pre
                    duration += 1
                    self.controller.put_action(drpy_tmp)
                if duration == 100:
                    self.drpy_pre = drpy
                    break

                self.controller.control_mujoco()
                self._torque, self.max_rotation = self.controller.write()
                for i in range(self.dof-1):
                    self.data.ctrl[i] = self._torque[i]
                mujoco.mj_step(self.model, self.data)


                if self.rendering:
                    self.render()

        ee = self.controller.get_ee()
        obs = self._observation(ee)
        done = self._done()
        reward = self._reward(action_rotation, done)
        info = self._info()

        self.drpy_pre = drpy
        self.action_pre = action_rotation



        return obs, reward, done, info

    def _observation(self, end_effector):

        # stack observations
        self.obs_q[1:] = self.obs_q[:-1]
        self.obs_xyz[1:] = self.obs_xyz[:-1]
        self.obs_rpy[1:] = self.obs_rpy[:-1]

        q_unscaled = self.data.qpos[0:self.k]
        q = (q_unscaled-self.q_range[:,0]) / (self.q_range[:,1] - self.q_range[:,0]) * (1-(-1)) - 1
        dq_unscaled = self.data.qvel[0:self.k]
        dq = (dq_unscaled - self.qdot_range[:, 0]) / (self.qdot_range[:, 1] - self.qdot_range[:, 0]) * (1 - (-1)) - 1

        xyz = end_effector[1][:3]
        rpy = end_effector[1][3:6]
        r6d = tools.orientation_euler_to_6d(rpy)

        self.obs_xyz[0] = xyz
        self.obs_rpy[0] = r6d
        self.obs_q[0] = q

        observation = dict(object=self.obs_object,q=self.obs_q,rpy=self.obs_rpy, x_pos=self.obs_xyz)
        # self.save_frame_data(end_effector)
        observation = self._flatten_obs(observation)



        return observation
    def _reward(self, action, done):
        if (self.action_pre == 0.0).all():
            self.action_pre = action
        reward_acc = -sum(abs(rotations.subtract_euler(tools.orientation_6d_to_euler(action),
                                                                  tools.orientation_6d_to_euler(self.action_pre))))

        reward_grasp = 0
        reward_contact = 0
        reward_bound = 0

        if self.control_mode == RL_CIRCULAR_CONTROL: # 잡은 이후
            if not -1 in self.contact_list:
                reward_grasp = -2+len(self.grasp_list) # grasp_list max = 8 : finger parts.
        if self.action_reset:
            reward_bound = -1
        if done:
            if self.contact_done:
                reward_contact = -1


        reward = self.rw_acc*reward_acc\
                 +self.rw_gr*reward_grasp\
                 +self.rw_c*reward_contact\
                 +self.rw_b*reward_bound

        return reward


    def _done(self):

        self.contact_list = tools.detect_contact(self.data.contact, self.desired_contact_bid)
        self.grasp_list = tools.detect_grasp(self.data.contact, self.obj, self.desired_contact_finger_bid, self.desired_contact_obj_bid)
        self.q_operation_list = tools.detect_q_operation(self.data.qpos, self.q_range)

        self.time_done = self.data.time - self.start_time >= self.episode_time
        self.reset_done = self.cnt_reset >= 5
        self.contact_done = -1 in self.contact_list
        self.bound_done = -1 in self.q_operation_list

        self.handle_angle = max(abs(self.data.qpos[-2:])) + self.handle_angle_reset
        self.goal_done = abs(self.required_angle - self.handle_angle) < 0.01
        if self.time_done or self.contact_done or self.goal_done or self.bound_done or self.reset_done:
            # print("contact :", self.contact_done, "  //joint :", self.bound_done, "  //time :", self.time_done)
            # print("epispde time : ",self.episode_time, "time:",self.data.time-self.start_time)

            return True
        else:
            return False
    def _info(self):
        info = {
            "collision": self.contact_done,
            "bound" : self.bound_done,
        }
        return info
    def _construct_action_space(self):
        action_space = 6
        action_low = -1*np.ones(action_space)
        action_high = 1* np.ones(action_space)
        return gym.spaces.Box(low=action_low, high=action_high, dtype=np.float32)

    def _construct_observation_space(self):

        s = {
            'object': spaces.Box(shape=(1, 13), low=-np.inf, high=np.inf, dtype=np.float32),
            'q': spaces.Box(shape=(self.stack, self.k), low=-1, high=1, dtype=np.float32),
            'rpy': spaces.Box(shape=(self.stack, 6), low=-1, high=1, dtype=np.float_),
            'x_pos': spaces.Box(shape=(self.stack, 3), low=-np.inf, high=np.inf, dtype=np.float_),
        }
        observation = spaces.Dict(s)
        observation.shape = 0
        for _, v in s.items():
            observation.shape += v.shape[0] * v.shape[1]
        return observation
    def render(self):
        if self.viewer is None:
            self.viewer = viewer.launch_passive(model=self.model, data=self.data)
        else:
            self.viewer.sync()
    def _flatten_obs(self, observation):
        flatten_obs = []
        for k,v in observation.items():
            flatten_obs = np.concatenate([flatten_obs, v.flatten()])
        return flatten_obs
    def env_randomization(self):

        obj_list = ["handle", "valve"]
        radius_list = [0.119, 0.1]
        o = randint(0,1)
        # o = 1 #valve 대상으로 한 코드는 아직 (x)
        obj = obj_list[o]
        radius = radius_list[o]
        # random_pos = [(random() * 0.4 + 0.3), (random()*0.8 - 0.4), random() * 0.7 + 0.1]
        #[0.3, 0.7] [-0.4, 0.4] [0.1. 0.8]
        handle_quat_candidate = [[0.25192415, -0.64412663, 0.57897236, 0.4317709],
                                 [-0.49077636, 0.42062713, -0.75930974, 0.07523369],
                                 [0.474576307745582, -0.089013785474907, 0.275616460318178, 0.831197594392378],
                                 [0., -0.707, 0.707, 0.],
                                 [-0.46086475, -0.63305975, 0.39180338, 0.48304156],
                                 [-0.07865809, -0.89033475, 0.16254433, -0.41796684],
                                 [0.70738827, 0., 0., 0.70682518]]
        handle_pos_candidate = [[0.52, 0, 0.8],
                                [0.28, -0.3, 0.8],
                                [0.326, 0.232, 0.559 + 0.35],
                                [0.55, 0., 0.75],
                                [0.4, 0.3, 0.5],
                                [0.25, 0.25, 0.9],
                                [0.48, 0, 0.9]]
        valve_quat_candidate = [[0., 1., 0., 0.],
                                [-0.707, 0.707, 0., 0.],
                                [0., -0.707, 0.707, 0.],
                                [0., -0.707, 0.707, 0.],
                                [0., 0.707, - 0., 0.707],
                                [-0.707, 0.707, 0., 0.],
                                [0., 1., 0., 0.]]
        valve_pos_candidate = [[0.38, 0., 0.45],
                               [0.3, 0.2, 0.6],
                               [0.28, -0.2, 0.7],
                               [0.30, 0.3, 0.5],
                               [0.28, 0.2, 0.55],
                               [0.3, 0.3, 0.6],
                               [0.3, 0.3, 0.6]]
        if obj == "handle":
            nobj = "valve"
            quat_candidate = handle_quat_candidate
            pos_candidate = handle_pos_candidate
        elif obj == "valve":
            nobj = "handle"
            quat_candidate = valve_quat_candidate
            pos_candidate = valve_pos_candidate

        bid = mujoco.mj_name2id(self.model, BODY, obj)
        nbid = mujoco.mj_name2id(self.model, BODY, nobj)

        if self.env_rand:
            i = randint(0, 6)
            axis = ['x', 'y', 'z']

            add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5))
            ori_quat = R.from_quat(tools.quat2xyzw(quat_candidate[i]))
            new_quat = add_quat * ori_quat
            random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()

            add_pos = [(random() - 0.5) / 5, (random() - 0.5) / 5, (random() - 0.5) / 5]
            random_pos = [x + y for x, y in zip(add_pos, pos_candidate[i])]
            # random_pos = [(random() * 0.4 + 0.3), (random()*0.8 - 0.4), random() * 0.7 + 0.1]
            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            # print("quat:",random_quat, "pos: ",random_pos)
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))


        else:

            i = self.episode_number if self.episode_number <= 6 else self.episode_number - 7
            # print(i)
            # i = 4
            self.direction = "cclk"
            random_quat = quat_candidate[i]
            random_pos = pos_candidate[i]


            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))
            # random_quat = self.model.body_quat[bid].copy().tolist()
            # random_pos =  self.model.body_pos[bid].copy().tolist()
            # r = R.from_quat(tools.quat2xyzw(random_quat))

        mujoco.mj_step(self.model, self.data)
        self.obj = obj
        obj_rotation6d = tools.orientation_quat_to_6d(self.model.body_quat[bid], "mujoco")

        if self.direction == "clk":
            classifier = self.classifier_clk
            direction = -1
        elif self.direction == "cclk":
            classifier = self.classifier_cclk
            direction = +1

        if obj == "handle":
            obj_id = [1,0,1]
            input_data = random_quat + random_pos
            test_input_data = torch.Tensor(input_data).cuda()
            predictions = classifier(test_input_data)
            angles = [4, 5, 6, 7, 8, 13, 14, 15, 16, 17, 22, 23, 24, 25, 26, 31, 32, 33, 34, 35]
            result = torch.argmax(predictions)
            result = angles[result]

            self.o_margin = [[0], [0.149], [0]]
            self.T_vv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
            # print("direction :", self.direction, "input:",input_data)
            # print("result :", torch.argmax(predictions), "angles :", result, "output:",predictions)

        elif obj == "valve":
            obj_id = [0, 1, 0]
            result = 0
            self.o_margin = [[0], [0], [-0.017]]
            self.T_vv = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        # result = 17
        init_angle = 2*np.pi*result/36
        self.obs_object = np.concatenate([self.model.body_pos[bid], obj_rotation6d, [direction], obj_id],
                                         axis=0)
        self.obs_object = self.obs_object.reshape((1, 13))


        self.obj_pos = random_pos
        self.obj_rotation = r.as_matrix()
        self.normal_vector = self.obj_rotation @ self.o_margin
        self.obj_normal = [0, 0, 0]
        for idx in range(3):
            self.obj_normal[idx] = self.normal_vector[idx][0] + self.obj_pos[idx]

        return r.as_matrix().tolist(), obj, radius, init_angle

    def save_frame_data(self, ee):
        r = R.from_euler('xyz', ee[1][3:6], degrees=False)
        rpyfromvalve_rot = r.inv() * R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)
        ee_align = R.from_euler('z', 45, degrees=True)
        rpyfromvalve = (ee_align * rpyfromvalve_rot).as_matrix()

        xyzfromvalve_rot = (R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)).as_matrix()
        xyzfromvalve_rot = np.concatenate([xyzfromvalve_rot, [[0, 0, 0]]], axis=0)
        xyzfromvalve_rot = np.concatenate(
            [xyzfromvalve_rot, [[self.obj_normal[0]], [self.obj_normal[1]], [self.obj_normal[2]], [1]]], axis=1)

        xyzfromvalve = inv(xyzfromvalve_rot) @ np.array([[ee[1][0]], [ee[1][1]], [ee[1][2]], [1]])

        if len(self.rpyfromvalve_data) == 0:
            self.rpyfromvalve_data = rpyfromvalve.reshape(1, 3, 3)
            self.xyzfromvalve_data = xyzfromvalve[0:3].reshape(1, 3)
            self.gripper_data = ee[2]
        else:
            self.rpyfromvalve_data = np.concatenate([self.rpyfromvalve_data, [rpyfromvalve]], axis=0)
            self.xyzfromvalve_data = np.concatenate([self.xyzfromvalve_data, [xyzfromvalve[0:3].reshape(3)]], axis=0)
            self.gripper_data = np.concatenate([self.gripper_data, ee[2]], axis=0)


    def read_file(self):
        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dr_heuristic.txt', 'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dr = list(map(float, f_list))
        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dp_heuristic.txt', 'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dp = list(map(float, f_list))

        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dy_heuristic.txt', 'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dy = list(map(float, f_list))

class fr3_valve_size:
    metadata = {"render_modes": ["human"], "render_fps": 30}
    def __init__(self) -> None:
        self.k = 7  # for jacobian calculation
        self.dof = 9  # all joints (include gripper joint)
        self.model_path = "/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/scene_valve.xml"
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.controller = controller.CController(self.k)
        self._torque = np.zeros(self.dof, dtype=np.float64)
        self.stack = 5
        self.rendering = True
        self.train = False
        self.env_rand = False
        self.observation_space = self._construct_observation_space()
        self.action_space = self._construct_action_space()
        ## reward weight
        self.reward_range = None
        self.rw_acc = 3  # np.exp(-sum(abs(action - self.action_pre)))
        self.rw_c = 10  # contact done -> -1
        self.rw_b = 1  # joint boundary done -> -1
        self.rw_gr = 1  # 1/-1 grasp


        self.viewer = None
        self.q_range = self.model.jnt_range[:self.k]
        self.qdot_range = np.array([[-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750],
                                    [-2.61, 2.61], [-2.61, 2.61], [-2.61, 2.61]])
        self.qdot_init = [0,0,0,0,0,0,0,0,0,0,0]
        self.q_init = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45),0,0,0,0]
        self.q_reset = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45),0.03,0.03,0,0]
        self.episode_number = -1

        self.classifier_clk = BringClassifier(
            "/home/kist-robot2/catkin_ws/src/franka_emika_panda/py_src/classifier/model_clk.pt")
        self.classifier_cclk = BringClassifier(
            "/home/kist-robot2/catkin_ws/src/franka_emika_panda/py_src/classifier/model_cclk.pt")
        desired_contact_list = ["finger_contact0", "finger_contact1",
                                "finger_contact2", "finger_contact3", "finger_contact4", "finger_contact5",
                                "finger_contact6", "finger_contact7",
                                "finger_contact8", "finger_contact9", "finger0_contact", "finger1_contact",
                                "valve_contact0", "valve_contact1",
                                "handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                "handle_contact4", "handle_contact5", "handle_contact6", "handle_contact7",
                                "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                "handle_contact15", "handle_contact16", "handle_contact18", "handle_contact19",
                                "handle_contact21", "handle_contact22", "handle_contact23"]
        desired_contact_list_finger = ["finger_contact1",
                                       "finger_contact2", "finger_contact3", "finger_contact4",
                                       "finger_contact6", "finger_contact7",
                                       "finger_contact8", "finger_contact9", ]
        desired_contact_list_obj = ["handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                 "handle_contact5", "handle_contact6",
                                "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                "handle_contact15", "handle_contact16",
                                "handle_contact21", "handle_contact22", "handle_contact23", "valve_contact0"]



        self.desired_contact_bid = tools.name2id(self.model, GEOM, desired_contact_list)
        self.desired_contact_finger_bid = tools.name2id(self.model, GEOM, desired_contact_list_finger)
        self.desired_contact_obj_bid = tools.name2id(self.model, GEOM, desired_contact_list_obj)

        self.direction_state = ["clk","cclk"]
        self.scale = 7

    def reset(self, direction=None):
        self.control_mode = 0
        # self.manipulability = []
        self.direction = direction
        cnt_frame = 0
        env_reset = True
        self.episode_time_elapsed = 0.0
        self.handle_angle = 0.0
        self.handle_angle_reset = 0.0
        self.action_reset = 0
        self.cnt_reset = 0
        self.drpy_data = []
        self.ddrpy_data = []
        self.obs_data = []
        while env_reset:
            if self.episode_number%10 == 0:
                self.scale = self.mujoco_xml()
                self.model_path = "/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/scene_valve.xml"
                self.model = mujoco.MjModel.from_xml_path(self.model_path)
                self.data = mujoco.MjData(self.model)

            self.episode_number += 1
            self.start_time = self.data.time + 1
            self.controller.initialize()
            self.data.qpos = self.q_init
            self.data.qvel = self.qdot_init

            if self.direction is None:
                self.direction = self.direction_state[randint(0, 1)]

            r, obj, radius, init_angle = self.env_randomization(scale=self.scale) #self.obs_object initialize


            self.init_angle = init_angle

            if self.direction == "clk":
                self.goal_angle = init_angle - 5* np.pi
            elif self.direction == "cclk":
                self.goal_angle = init_angle + 5 * np.pi
            self.required_angle = abs(self.goal_angle - self.init_angle)

            self.episode_time = abs( MOTION_TIME_CONST * abs(self.goal_angle-self.init_angle) * radius)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
            self.controller.randomize_env2(r, obj, self.scale, self.data.xpos[:22].reshape(66, ), self.init_angle, self.goal_angle, RL, RPY)

            self.controller.control_mujoco()


            self.contact_done = False
            self.bound_done = False
            self.goal_done = False
            self.reset_done = False
            self.action_pre  = np.zeros(6)
            self.drpy_pre  = np.zeros(3)

            self.obs_q = np.zeros([self.stack, self.k])
            self.obs_rpy = np.zeros([self.stack,6])
            self.obs_xyz = np.zeros([self.stack, 3])

            self.rpyfromvalve_data = []
            self.path_data = []

            while self.control_mode != 4:
                self.control_mode = self.controller.control_mode()
                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
                self.controller.control_mujoco()
                self._torque, self.max_rotation = self.controller.write()
                for i in range(self.dof - 1):
                    self.data.ctrl[i] = self._torque[i]

                mujoco.mj_step(self.model, self.data)
                ee = self.controller.get_ee()

                self.path_data.append([ee[1] + self.data.qpos[:7].tolist()])
                done = self._done()
                normalized_q = self.obs_q[0]
                if max(abs(normalized_q)) > 0.95:
                    done = True

                if done:
                    # np.save("path_data.npy", self.path_data)
                    break
                # --- collect observation for initialization ---
                if cnt_frame == 100:
                    cnt_frame = 0
                    end_effector = self.controller.get_ee()
                    # self.save_frame_data(end_effector)
                    obs = self._observation(end_effector)
                cnt_frame += 1
                if self.rendering:
                    self.render()
            if self.control_mode == 4:

                env_reset = False
                self.start_time = self.data.time
                self.q_reset[:self.k] = self.data.qpos[:self.k]

        end_effector = self.controller.get_ee()
        self.action_pre = tools.orientation_euler_to_6d(end_effector[0][3:6])
        return obs

    def step(self, action_rotation):
        drpy = tools.orientation_6d_to_euler(action_rotation)
        done = False
        duration = 0


        # if not self.train: # 학습하는 동안에는 reset없애고
        #     normalized_q = self.obs_q[0]
        #     if max(abs(normalized_q)) > 0.95:
        #         self.action_reset = 1
        #         self.cnt_reset += 1
        #     else:
        #         self.action_reset = 0

        if self.action_reset:
            self.handle_angle_reset += max(abs(self.data.qpos[-2:]))
            self.data.qpos= self.q_reset
            self.data.qvel = self.qdot_init
            mujoco.mj_step(self.model, self.data)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
            self.controller.target_replan()
            if self.rendering:
                    self.render()
        else:
            while not done:
                done = self._done()
                self.control_mode = self.controller.control_mode()
                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))

                # --- RL controller input ---
                if self.control_mode == RL_CIRCULAR_CONTROL:
                    drpy_tmp = (drpy - self.drpy_pre) / 100 * duration + self.drpy_pre
                    duration += 1
                    self.controller.put_action(drpy_tmp)
                if duration == 100:
                    self.drpy_pre = drpy
                    break

                self.controller.control_mujoco()
                self._torque, self.max_rotation = self.controller.write()
                for i in range(self.dof-1):
                    self.data.ctrl[i] = self._torque[i]
                mujoco.mj_step(self.model, self.data)


                if self.rendering:
                    self.render()

        ee = self.controller.get_ee()
        obs = self._observation(ee)
        done = self._done()
        reward = self._reward(action_rotation, done)
        info = self._info()

        self.drpy_pre = drpy
        self.action_pre = action_rotation



        return obs, reward, done, info

    def _observation(self, end_effector):

        # stack observations
        self.obs_q[1:] = self.obs_q[:-1]
        self.obs_xyz[1:] = self.obs_xyz[:-1]
        self.obs_rpy[1:] = self.obs_rpy[:-1]

        q_unscaled = self.data.qpos[0:self.k]
        q = (q_unscaled-self.q_range[:,0]) / (self.q_range[:,1] - self.q_range[:,0]) * (1-(-1)) - 1
        dq_unscaled = self.data.qvel[0:self.k]
        dq = (dq_unscaled - self.qdot_range[:, 0]) / (self.qdot_range[:, 1] - self.qdot_range[:, 0]) * (1 - (-1)) - 1

        xyz = end_effector[1][:3]
        rpy = end_effector[1][3:6]
        r6d = tools.orientation_euler_to_6d(rpy)

        self.obs_xyz[0] = xyz
        self.obs_rpy[0] = r6d
        self.obs_q[0] = q

        observation = dict(object=self.obs_object,q=self.obs_q,rpy=self.obs_rpy, x_pos=self.obs_xyz)
        # self.save_frame_data(end_effector)
        observation = self._flatten_obs(observation)
        # jacobian = np.array(self.controller.get_jacobian())
        #
        # self.manipulability.append(tools.calc_manipulability(jacobian).tolist())
        self.drpy_data.append(end_effector[0][3:6])
        self.obs_data.append(observation)
        self.ddrpy_data.append((rotations.subtract_euler(np.array(end_effector[0][3:6]),
                                                         np.array(self.drpy_pre))))
        self.drpy_pre = end_effector[0][3:6]
        return observation
    def _reward(self, action, done):
        # if (self.action_pre == 0.0).all():
        #     self.action_pre = action
        reward_acc = -sum(abs(rotations.subtract_euler(tools.orientation_6d_to_euler(action),
                                                                  tools.orientation_6d_to_euler(self.action_pre))))


        reward_grasp = 0
        reward_contact = 0
        reward_bound = 0

        if self.control_mode == RL_CIRCULAR_CONTROL: # 잡은 이후
            if not -1 in self.contact_list:
                # reward_grasp = -2+len(self.grasp_list) # grasp_list max = 8 : finger parts.
                reward_grasp = -4+len(self.grasp_list) # grasp_list max = 8 : finger parts.
        if self.action_reset:
            reward_bound = -1
        if done:
            if self.contact_done:
                reward_contact = -1


        reward = self.rw_acc*reward_acc\
                 +self.rw_gr*reward_grasp\
                 +self.rw_c*reward_contact\
                 +self.rw_b*reward_bound

        return reward


    def _done(self):

        self.contact_list = tools.detect_contact(self.data.contact, self.desired_contact_bid)
        self.grasp_list = tools.detect_grasp(self.data.contact, self.obj, self.desired_contact_finger_bid, self.desired_contact_obj_bid)
        self.q_operation_list = tools.detect_q_operation(self.data.qpos, self.q_range)

        self.time_done = self.data.time - self.start_time >= self.episode_time
        self.reset_done = self.cnt_reset >= 5
        self.contact_done = -1 in self.contact_list
        self.bound_done = -1 in self.q_operation_list

        self.handle_angle = max(abs(self.data.qpos[-2:])) + self.handle_angle_reset
        self.goal_done = abs(self.required_angle - self.handle_angle) < 0.01
        if self.time_done or self.contact_done or self.goal_done or self.bound_done or self.reset_done:
            # print(self.control_mode)
            # print("contact :", self.contact_done, "  //joint :", self.bound_done, "  //time :", self.time_done)
            # print("epispde time : ",self.episode_time, "time:",self.data.time-self.start_time)
            # np.save("/home/kist-robot2/catkin_ws/src/franka_overall/py_src/m0.npy", self.manipulability)
            # np.save("obs_3.npy", self.obs_data)
            np.save("drpy_2.npy", self.drpy_data)
            np.save("ddrpy_2.npy", self.ddrpy_data)
            return True
        else:
            return False
    def _info(self):
        info = {
            "collision": self.contact_done,
            "bound" : self.bound_done,
        }
        return info
    def _construct_action_space(self):
        action_space = 6
        action_low = -1*np.ones(action_space)
        action_high = 1* np.ones(action_space)
        return gym.spaces.Box(low=action_low, high=action_high, dtype=np.float32)

    def _construct_observation_space(self):

        s = {
            'object': spaces.Box(shape=(1, 11), low=-np.inf, high=np.inf, dtype=np.float32),
            'q': spaces.Box(shape=(self.stack, self.k), low=-1, high=1, dtype=np.float32),
            'rpy': spaces.Box(shape=(self.stack, 6), low=-1, high=1, dtype=np.float_),
            'x_pos': spaces.Box(shape=(self.stack, 3), low=-np.inf, high=np.inf, dtype=np.float_),
        }
        observation = spaces.Dict(s)
        observation.shape = 0
        for _, v in s.items():
            observation.shape += v.shape[0] * v.shape[1]
        return observation
    def render(self):
        if self.viewer is None:
            self.viewer = viewer.launch_passive(model=self.model, data=self.data)
        else:
            self.viewer.sync()
    def _flatten_obs(self, observation):
        flatten_obs = []
        for k,v in observation.items():
            flatten_obs = np.concatenate([flatten_obs, v.flatten()])
        return flatten_obs
    def env_randomization(self, scale):

        obj = "handle"
        radius = 0.017 * scale
        handle_quat_candidate = [[0.25192415, -0.64412663, 0.57897236, 0.4317709],
                                 [-0.49077636, 0.42062713, -0.75930974, 0.07523369],
                                 [0.474576307745582, -0.089013785474907, 0.275616460318178, 0.831197594392378],
                                 [0., -0.707, 0.707, 0.],
                                 [-0.46086475, -0.63305975, 0.39180338, 0.48304156],
                                 [-0.07865809, -0.89033475, 0.16254433, -0.41796684],
                                 [0.70738827, 0., 0., 0.70682518],
                                 [0.70710678, 0.70710678, 0. ,        0.       ]]
        handle_pos_candidate = [[0.52, 0, 0.8],
                                [0.28, -0.3, 0.8],
                                [0.326, 0.232, 0.559 + 0.35],
                                [0.55, 0., 0.75],
                                [0.4, 0.3, 0.5],
                                [0.25, 0.25, 0.9],
                                [0.48, 0, 0.9],
                                [0.580994101778967, -0.045675755104744684, 0.115]]

        nobj = "valve"
        quat_candidate = handle_quat_candidate
        pos_candidate = handle_pos_candidate

        bid = mujoco.mj_name2id(self.model, BODY, obj)
        nbid = mujoco.mj_name2id(self.model, BODY, nobj)

        if self.env_rand:
            i = randint(0, 7)
            axis = ['x', 'y', 'z']

            # add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5))
            # ori_quat = R.from_quat(tools.quat2xyzw(quat_candidate[i]))
            # new_quat = add_quat * ori_quat
            # random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()
            add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5))
            ori_quat = R.from_quat(tools.quat2xyzw(handle_quat_candidate[i]))
            y_rot = R.from_euler('y', random() * 360, degrees=True)
            new_quat = ori_quat * add_quat * y_rot
            random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()

            add_pos = [(random() - 0.5) / 5, (random() - 0.5) / 5, (random() - 0.5) / 5]
            random_pos = [x + y for x, y in zip(add_pos, pos_candidate[i])]
            if random_pos[2] < 0.01:
                random_pos[2] = 0.01
            # random_pos = [(random() * 0.4 + 0.3), (random()*0.8 - 0.4), random() * 0.7 + 0.1]
            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            # print("quat:",random_quat, "pos: ",random_pos)
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))


        else:

            i = self.episode_number if self.episode_number <= 7 else self.episode_number - 8
            # print(i)
            # i = 3

            random_quat = quat_candidate[i]
            random_pos = pos_candidate[i]
            # self.direction = "cclk"
            # random_quat = [0.70710678, 0.70710678 ,0.,         0.    ]  # [0,0,0,1]
            # random_pos = [ 0.580994101778967, -0.045675755104744684, 0.115]
            # random_pos = [0.52, 0, 0.8]
            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))
            # random_quat = self.model.body_quat[bid].copy().tolist()
            # random_pos =  self.model.body_pos[bid].copy().tolist()
            # r = R.from_quat(tools.quat2xyzw(random_quat))

        mujoco.mj_step(self.model, self.data)
        self.obj = obj
        obj_rotation6d = tools.orientation_quat_to_6d(self.model.body_quat[bid], "mujoco")

        if self.direction == "clk":
            classifier = self.classifier_clk
            direction = -1
        elif self.direction == "cclk":
            classifier = self.classifier_cclk
            direction = +1

        input_data = random_quat + random_pos + [radius]
        test_input_data = torch.Tensor(input_data).cuda()
        predictions = classifier(test_input_data)
        angles = [4, 5, 6, 7, 8, 13, 14, 15, 16, 17, 22, 23, 24, 25, 26, 31, 32, 33, 34, 35]
        result = torch.argmax(predictions)
        result = angles[result]
        # result = 0
        self.o_margin = [[0], [0.0213 * scale], [0]]
        self.T_vv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        # print("direction :", self.direction, "input:",input_data)
        # print("result :", torch.argmax(predictions), "angles :", result, "output:",predictions)

        init_angle = 2 * np.pi * result / 36
        self.obs_object = np.concatenate([[radius], self.model.body_pos[bid], obj_rotation6d, [direction]],
                                         axis=0)
        self.obs_object = self.obs_object.reshape((1, 11))

        self.obj_pos = random_pos
        self.obj_rotation = r.as_matrix()
        self.normal_vector = self.obj_rotation @ self.o_margin
        self.obj_normal = [0, 0, 0]
        for idx in range(3):
            self.obj_normal[idx] = self.normal_vector[idx][0] + self.obj_pos[idx]

        return r.as_matrix().tolist(), obj, radius, init_angle

    def save_frame_data(self, ee):
        r = R.from_euler('xyz', ee[1][3:6], degrees=False)
        rpyfromvalve_rot = r.inv() * R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)
        ee_align = R.from_euler('z', 45, degrees=True)
        rpyfromvalve = (ee_align * rpyfromvalve_rot).as_matrix()

        xyzfromvalve_rot = (R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)).as_matrix()
        xyzfromvalve_rot = np.concatenate([xyzfromvalve_rot, [[0, 0, 0]]], axis=0)
        xyzfromvalve_rot = np.concatenate(
            [xyzfromvalve_rot, [[self.obj_normal[0]], [self.obj_normal[1]], [self.obj_normal[2]], [1]]], axis=1)

        xyzfromvalve = inv(xyzfromvalve_rot) @ np.array([[ee[1][0]], [ee[1][1]], [ee[1][2]], [1]])

        if len(self.rpyfromvalve_data) == 0:
            self.rpyfromvalve_data = rpyfromvalve.reshape(1, 3, 3)
            self.xyzfromvalve_data = xyzfromvalve[0:3].reshape(1, 3)
            self.gripper_data = ee[2]
        else:
            self.rpyfromvalve_data = np.concatenate([self.rpyfromvalve_data, [rpyfromvalve]], axis=0)
            self.xyzfromvalve_data = np.concatenate([self.xyzfromvalve_data, [xyzfromvalve[0:3].reshape(3)]], axis=0)
            self.gripper_data = np.concatenate([self.gripper_data, ee[2]], axis=0)


    def read_file(self):
        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dr_heuristic.txt', 'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dr = list(map(float, f_list))
        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dp_heuristic.txt', 'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dp = list(map(float, f_list))

        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dy_heuristic.txt', 'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dy = list(map(float, f_list))

    def mujoco_xml(self):
        if self.rendering:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None

        del self.model
        del self.data
        s = randint(5,9)
        # s=7
        handle_xml = f'''
            <mujocoinclude>
                <compiler angle="radian" meshdir="meshes/" autolimits="true"/>
                <size njmax="500" nconmax="100" />
                <visual>
                    <global offwidth="3024" offheight="1680" />
                    <quality shadowsize="4096" offsamples="8" />
                    <map force="0.1" fogend="5" />
                </visual>
                
                
                <asset>
                
                    <mesh name="handle_base" file="objects/handle_base.STL" scale="{s} {s} {s}"/>
                    <mesh name="handle_base0" file="objects/handle_base/handle_base000.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle_base1" file="objects/handle_base/handle_base001.obj" scale="{s} {s} {s}"/>
                    
                    <mesh name="handle" file="objects/handle.STL" scale="{s} {s} {s}"/>
                    
                    
                    <mesh name="handle0" file="objects/handle2/handle000.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle1" file="objects/handle2/handle001.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle2" file="objects/handle2/handle002.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle3" file="objects/handle2/handle003.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle4" file="objects/handle2/handle004.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle5" file="objects/handle2/handle005.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle6" file="objects/handle2/handle006.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle7" file="objects/handle2/handle007.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle8" file="objects/handle2/handle008.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle9" file="objects/handle2/handle009.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle10" file="objects/handle2/handle010.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle11" file="objects/handle2/handle011.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle12" file="objects/handle2/handle012.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle13" file="objects/handle2/handle013.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle14" file="objects/handle2/handle014.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle15" file="objects/handle2/handle015.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle16" file="objects/handle2/handle016.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle17" file="objects/handle2/handle017.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle18" file="objects/handle2/handle018.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle19" file="objects/handle2/handle019.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle20" file="objects/handle2/handle020.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle21" file="objects/handle2/handle021.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle22" file="objects/handle2/handle022.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle23" file="objects/handle2/handle023.obj" scale="{s} {s} {s}"/>
                </asset>
                
                <contact>
                    <exclude name="handle_contact" body1="handle_base" body2="handle_handle"/>
                </contact>
            
            </mujocoinclude>
        '''

        # Now you can write the XML content to a file
        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/assets_handle.xml', 'w') as file:
            file.write(handle_xml)
        return s


class fr3_smooth_start:
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self) -> None:
        self.k = 7  # for jacobian calculation
        self.dof = 9  # all joints (include gripper joint)
        self.model_path = "/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/scene_valve.xml"
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.controller = controller.CController(self.k)
        self._torque = np.zeros(self.dof, dtype=np.float64)
        self.stack = 5
        self.rendering = True
        self.train = False
        self.env_rand = False
        self.observation_space = self._construct_observation_space()
        self.action_space = self._construct_action_space()
        ## reward weight
        self.reward_range = None
        self.rw_acc = 3  # np.exp(-sum(abs(action - self.action_pre)))
        self.rw_c = 10  # contact done -> -1
        self.rw_b = 1  # joint boundary done -> -1
        self.rw_gr = 1  # 1/-1 grasp

        self.viewer = None
        self.q_range = self.model.jnt_range[:self.k]
        self.qdot_range = np.array([[-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750], [-2.1750, 2.1750],
                                    [-2.61, 2.61], [-2.61, 2.61], [-2.61, 2.61]])
        self.qdot_init = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.q_init = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45), 0, 0, 0, 0]
        self.q_reset = [0, np.deg2rad(-60), 0, np.deg2rad(-90), 0, np.deg2rad(90), np.deg2rad(45), 0.03, 0.03, 0, 0]
        self.episode_number = -1

        self.classifier_clk = BringClassifier(
            "/home/kist-robot2/catkin_ws/src/franka_emika_panda/py_src/classifier/model_clk.pt")
        self.classifier_cclk = BringClassifier(
            "/home/kist-robot2/catkin_ws/src/franka_emika_panda/py_src/classifier/model_cclk.pt")
        desired_contact_list = ["finger_contact0", "finger_contact1",
                                "finger_contact2", "finger_contact3", "finger_contact4", "finger_contact5",
                                "finger_contact6", "finger_contact7",
                                "finger_contact8", "finger_contact9", "finger0_contact", "finger1_contact",
                                "valve_contact0", "valve_contact1",
                                "handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                "handle_contact4", "handle_contact5", "handle_contact6", "handle_contact7",
                                "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                "handle_contact15", "handle_contact16", "handle_contact18", "handle_contact19",
                                "handle_contact21", "handle_contact22", "handle_contact23"]
        desired_contact_list_finger = ["finger_contact1",
                                       "finger_contact2", "finger_contact3", "finger_contact4",
                                       "finger_contact6", "finger_contact7",
                                       "finger_contact8", "finger_contact9", ]
        desired_contact_list_obj = ["handle_contact0", "handle_contact1", "handle_contact2", "handle_contact3",
                                    "handle_contact5", "handle_contact6",
                                    "handle_contact8", "handle_contact10", "handle_contact11", "handle_contact12",
                                    "handle_contact15", "handle_contact16",
                                    "handle_contact21", "handle_contact22", "handle_contact23", "valve_contact0"]

        self.desired_contact_bid = tools.name2id(self.model, GEOM, desired_contact_list)
        self.desired_contact_finger_bid = tools.name2id(self.model, GEOM, desired_contact_list_finger)
        self.desired_contact_obj_bid = tools.name2id(self.model, GEOM, desired_contact_list_obj)

        self.direction_state = ["clk", "cclk"]
        self.scale = 7

    def reset(self, direction=None):
        self.control_mode = 0
        # self.manipulability = []
        self.direction = direction
        cnt_frame = 0
        env_reset = True
        self.episode_time_elapsed = 0.0
        self.handle_angle = 0.0
        self.handle_angle_reset = 0.0
        self.action_reset = 0
        self.cnt_reset = 0
        self.drpy_data = []
        self.ddrpy_data = []
        self.obs_data = []
        while env_reset:
            if self.episode_number % 10 == 0:
                self.scale = self.mujoco_xml()
                self.model_path = "/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/scene_valve.xml"
                self.model = mujoco.MjModel.from_xml_path(self.model_path)
                self.data = mujoco.MjData(self.model)

            self.episode_number += 1
            self.start_time = self.data.time + 1
            self.controller.initialize()
            self.data.qpos = self.q_init
            self.data.qvel = self.qdot_init

            if self.direction is None:
                self.direction = self.direction_state[randint(0, 1)]

            r, obj, radius, init_angle = self.env_randomization(scale=self.scale)  # self.obs_object initialize

            self.init_angle = init_angle

            if self.direction == "clk":
                self.goal_angle = init_angle - 5 * np.pi
            elif self.direction == "cclk":
                self.goal_angle = init_angle + 5 * np.pi
            self.required_angle = abs(self.goal_angle - self.init_angle)

            self.episode_time = abs(MOTION_TIME_CONST * abs(self.goal_angle - self.init_angle) * radius)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
            self.controller.randomize_env2(r, obj, self.scale, self.data.xpos[:22].reshape(66, ), self.init_angle,
                                           self.goal_angle, RL, RPY)

            self.controller.control_mujoco()

            self.contact_done = False
            self.bound_done = False
            self.goal_done = False
            self.reset_done = False
            self.action_pre = np.zeros(6)
            self.drpy_pre = np.zeros(3)

            self.obs_q = np.zeros([self.stack, self.k])
            self.obs_rpy = np.zeros([self.stack, 6])
            self.obs_xyz = np.zeros([self.stack, 3])

            self.rpyfromvalve_data = []
            self.path_data = []

            while self.control_mode != 4:
                self.control_mode = self.controller.control_mode()
                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
                self.controller.control_mujoco()
                self._torque, self.max_rotation = self.controller.write()
                for i in range(self.dof - 1):
                    self.data.ctrl[i] = self._torque[i]

                mujoco.mj_step(self.model, self.data)
                ee = self.controller.get_ee()

                self.path_data.append([ee[1] + self.data.qpos[:7].tolist()])
                done = self._done()
                normalized_q = self.obs_q[0]
                if max(abs(normalized_q)) > 0.95:
                    done = True

                if done:
                    # np.save("path_data.npy", self.path_data)
                    break
                # --- collect observation for initialization ---
                if cnt_frame == 100:
                    cnt_frame = 0
                    end_effector = self.controller.get_ee()
                    # self.save_frame_data(end_effector)
                    obs = self._observation(end_effector)
                    self.drpy_pre = end_effector[0][3:6]
                cnt_frame += 1
                if self.rendering:
                    self.render()
            if self.control_mode == 4:
                env_reset = False
                self.start_time = self.data.time
                self.q_reset[:self.k] = self.data.qpos[:self.k]

        end_effector = self.controller.get_ee()
        self.action_pre = tools.orientation_euler_to_6d(end_effector[0][3:6])
        return obs

    def step(self, action_rotation):
        drpy = tools.orientation_6d_to_euler(action_rotation)
        done = False
        duration = 0

        # if not self.train: # 학습하는 동안에는 reset없애고
        #     normalized_q = self.obs_q[0]
        #     if max(abs(normalized_q)) > 0.95:
        #         self.action_reset = 1
        #         self.cnt_reset += 1
        #     else:
        #         self.action_reset = 0

        if self.action_reset:
            self.handle_angle_reset += max(abs(self.data.qpos[-2:]))
            self.data.qpos = self.q_reset
            self.data.qvel = self.qdot_init
            mujoco.mj_step(self.model, self.data)
            self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                 self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))
            self.controller.target_replan()
            if self.rendering:
                self.render()
        else:
            while not done:
                done = self._done()
                self.control_mode = self.controller.control_mode()
                self.controller.read(self.data.time, self.data.qpos[0:self.dof], self.data.qvel[0:self.dof],
                                     self.model.opt.timestep, self.data.xpos[:22].reshape(66, ))

                # --- RL controller input ---
                if self.control_mode == RL_CIRCULAR_CONTROL:
                    drpy_tmp = (drpy - self.drpy_pre) / 100 * duration + self.drpy_pre
                    duration += 1
                    self.controller.put_action(drpy_tmp)
                if duration == 100:
                    self.drpy_pre = drpy
                    break

                self.controller.control_mujoco()
                self._torque, self.max_rotation = self.controller.write()
                for i in range(self.dof - 1):
                    self.data.ctrl[i] = self._torque[i]
                mujoco.mj_step(self.model, self.data)

                if self.rendering:
                    self.render()

        ee = self.controller.get_ee()
        obs = self._observation(ee)
        done = self._done()
        reward = self._reward(action_rotation, done)
        info = self._info()

        self.drpy_pre = drpy
        self.action_pre = action_rotation

        return obs, reward, done, info

    def _observation(self, end_effector):

        # stack observations
        self.obs_q[1:] = self.obs_q[:-1]
        self.obs_xyz[1:] = self.obs_xyz[:-1]
        self.obs_rpy[1:] = self.obs_rpy[:-1]

        q_unscaled = self.data.qpos[0:self.k]
        q = (q_unscaled - self.q_range[:, 0]) / (self.q_range[:, 1] - self.q_range[:, 0]) * (1 - (-1)) - 1
        dq_unscaled = self.data.qvel[0:self.k]
        dq = (dq_unscaled - self.qdot_range[:, 0]) / (self.qdot_range[:, 1] - self.qdot_range[:, 0]) * (1 - (-1)) - 1

        xyz = end_effector[1][:3]
        rpy = end_effector[1][3:6]
        r6d = tools.orientation_euler_to_6d(rpy)

        self.obs_xyz[0] = xyz
        self.obs_rpy[0] = r6d
        self.obs_q[0] = q
        self.drot_pre = tools.orientation_euler_to_6d(end_effector[0][3:6])
        observation = dict(object=self.obs_object, drot_pre=self.drot_pre, q=self.obs_q, rpy=self.obs_rpy, x_pos=self.obs_xyz)
        # self.save_frame_data(end_effector)
        observation = self._flatten_obs(observation)
        # jacobian = np.array(self.controller.get_jacobian())
        #
        # self.manipulability.append(tools.calc_manipulability(jacobian).tolist())
        self.drpy_data.append(end_effector[0][3:6])
        self.obs_data.append(observation)
        self.ddrpy_data.append((rotations.subtract_euler(np.array(end_effector[0][3:6]),
                                                         np.array(self.drpy_pre))))

        return observation

    def _reward(self, action, done):
        # if (self.action_pre == 0.0).all():
        #     self.action_pre = action
        reward_acc = -sum(abs(rotations.subtract_euler(tools.orientation_6d_to_euler(action),
                                                       tools.orientation_6d_to_euler(self.action_pre))))

        reward_grasp = 0
        reward_contact = 0
        reward_bound = 0

        if self.control_mode == RL_CIRCULAR_CONTROL:  # 잡은 이후
            if not -1 in self.contact_list:
                reward_grasp = -2+len(self.grasp_list) # grasp_list max = 8 : finger parts.
                # reward_grasp = -4 + len(self.grasp_list)  # grasp_list max = 8 : finger parts.
        if self.action_reset:
            reward_bound = -1
        if done:
            if self.contact_done:
                reward_contact = -1

        reward = self.rw_acc * reward_acc \
                 + self.rw_gr * reward_grasp \
                 + self.rw_c * reward_contact \
                 + self.rw_b * reward_bound

        return reward

    def _done(self):

        self.contact_list = tools.detect_contact(self.data.contact, self.desired_contact_bid)
        self.grasp_list = tools.detect_grasp(self.data.contact, self.obj, self.desired_contact_finger_bid,
                                             self.desired_contact_obj_bid)
        self.q_operation_list = tools.detect_q_operation(self.data.qpos, self.q_range)

        self.time_done = self.data.time - self.start_time >= self.episode_time
        self.reset_done = self.cnt_reset >= 5
        self.contact_done = -1 in self.contact_list
        self.bound_done = -1 in self.q_operation_list

        self.handle_angle = max(abs(self.data.qpos[-2:])) + self.handle_angle_reset
        self.goal_done = abs(self.required_angle - self.handle_angle) < 0.01
        if self.time_done or self.contact_done or self.goal_done or self.bound_done or self.reset_done:
            # print(self.control_mode)
            # print("contact :", self.contact_done, "  //joint :", self.bound_done, "  //time :", self.time_done)
            # print("epispde time : ",self.episode_time, "time:",self.data.time-self.start_time)
            # np.save("/home/kist-robot2/catkin_ws/src/franka_overall/py_src/m0.npy", self.manipulability)
            # np.save("obs_3.npy", self.obs_data)
            np.save("drpy_smooth2.npy", self.drpy_data)
            np.save("ddrpy_smooth2.npy", self.ddrpy_data)
            return True
        else:
            return False

    def _info(self):
        info = {
            "collision": self.contact_done,
            "bound": self.bound_done,
        }
        return info

    def _construct_action_space(self):
        action_space = 6
        action_low = -1 * np.ones(action_space)
        action_high = 1 * np.ones(action_space)
        return gym.spaces.Box(low=action_low, high=action_high, dtype=np.float32)

    def _construct_observation_space(self):

        s = {
            'object': spaces.Box(shape=(1, 11), low=-np.inf, high=np.inf, dtype=np.float32),
            'drot_pre': spaces.Box(shape=(1, 6), low=-np.inf, high=np.inf, dtype=np.float32),
            'q': spaces.Box(shape=(self.stack, self.k), low=-1, high=1, dtype=np.float32),
            'rpy': spaces.Box(shape=(self.stack, 6), low=-1, high=1, dtype=np.float_),
            'x_pos': spaces.Box(shape=(self.stack, 3), low=-np.inf, high=np.inf, dtype=np.float_),
        }
        observation = spaces.Dict(s)
        observation.shape = 0
        for _, v in s.items():
            observation.shape += v.shape[0] * v.shape[1]
        return observation

    def render(self):
        if self.viewer is None:
            self.viewer = viewer.launch_passive(model=self.model, data=self.data)
        else:
            self.viewer.sync()

    def _flatten_obs(self, observation):
        flatten_obs = []
        for k, v in observation.items():
            flatten_obs = np.concatenate([flatten_obs, v.flatten()])
        return flatten_obs

    def env_randomization(self, scale):

        obj = "handle"
        radius = 0.017 * scale
        handle_quat_candidate = [[0.25192415, -0.64412663, 0.57897236, 0.4317709],
                                 [-0.49077636, 0.42062713, -0.75930974, 0.07523369],
                                 [0.474576307745582, -0.089013785474907, 0.275616460318178, 0.831197594392378],
                                 [0., -0.707, 0.707, 0.],
                                 [-0.46086475, -0.63305975, 0.39180338, 0.48304156],
                                 [-0.07865809, -0.89033475, 0.16254433, -0.41796684],
                                 [0.70738827, 0., 0., 0.70682518],
                                 [0.70710678, 0.70710678, 0. ,        0.       ]]
        handle_pos_candidate = [[0.52, 0, 0.8],
                                [0.28, -0.3, 0.8],
                                [0.326, 0.232, 0.559 + 0.35],
                                [0.55, 0., 0.75],
                                [0.4, 0.3, 0.5],
                                [0.25, 0.25, 0.9],
                                [0.48, 0, 0.9],
                                [0.580994101778967, -0.045675755104744684, 0.115]]

        nobj = "valve"
        quat_candidate = handle_quat_candidate
        pos_candidate = handle_pos_candidate

        bid = mujoco.mj_name2id(self.model, BODY, obj)
        nbid = mujoco.mj_name2id(self.model, BODY, nobj)

        if self.env_rand:
            i = randint(0, 7)
            axis = ['x', 'y', 'z']

            # add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5))
            # ori_quat = R.from_quat(tools.quat2xyzw(quat_candidate[i]))
            # new_quat = add_quat * ori_quat
            # random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()
            add_quat = R.from_euler(axis[randint(0, 2)], (random() - 0.5))
            ori_quat = R.from_quat(tools.quat2xyzw(handle_quat_candidate[i]))
            y_rot = R.from_euler('y', random() * 360, degrees=True)
            new_quat = ori_quat * add_quat * y_rot
            random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()

            add_pos = [(random() - 0.5) / 5, (random() - 0.5) / 5, (random() - 0.5) / 5]
            random_pos = [x + y for x, y in zip(add_pos, pos_candidate[i])]
            if random_pos[2] < 0.01:
                random_pos[2] = 0.01
            # random_pos = [(random() * 0.4 + 0.3), (random()*0.8 - 0.4), random() * 0.7 + 0.1]
            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            # print("quat:",random_quat, "pos: ",random_pos)
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))


        else:

            i = self.episode_number if self.episode_number <= 7 else self.episode_number - 8
            # print(i)
            # i = 3
            i=7
            random_quat = quat_candidate[i]
            random_pos = pos_candidate[i]
            self.direction = "clk"
            # random_quat = [0.70710678, 0.70710678 ,0.,         0.    ]  # [0,0,0,1]
            # random_pos = [ 0.580994101778967, -0.045675755104744684, 0.115]
            # random_pos = [0.52, 0, 0.8]

            # y_rot = R.from_euler('y', random() * 60, degrees=True)
            # ori_quat = R.from_quat(tools.quat2xyzw(random_quat))
            # new_quat = ori_quat * y_rot
            # random_quat = tools.xyzw2quat(new_quat.as_quat()).tolist()

            self.model.body_quat[bid] = random_quat
            self.model.body_pos[bid] = random_pos
            self.model.body_pos[nbid] += 3
            r = R.from_quat(tools.quat2xyzw(random_quat))
            # random_quat = self.model.body_quat[bid].copy().tolist()
            # random_pos =  self.model.body_pos[bid].copy().tolist()
            # r = R.from_quat(tools.quat2xyzw(random_quat))

        mujoco.mj_step(self.model, self.data)
        self.obj = obj
        obj_rotation6d = tools.orientation_quat_to_6d(self.model.body_quat[bid], "mujoco")

        if self.direction == "clk":
            classifier = self.classifier_clk
            direction = -1
        elif self.direction == "cclk":
            classifier = self.classifier_cclk
            direction = +1

        input_data = random_quat + random_pos + [radius]
        test_input_data = torch.Tensor(input_data).cuda()
        predictions = classifier(test_input_data)
        angles = [4, 5, 6, 7, 8, 13, 14, 15, 16, 17, 22, 23, 24, 25, 26, 31, 32, 33, 34, 35]
        result = torch.argmax(predictions)
        result = angles[result]
        # result = 0
        self.o_margin = [[0], [0.0213 * scale], [0]]
        self.T_vv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        # print("direction :", self.direction, "input:",input_data)
        # print("result :", torch.argmax(predictions), "angles :", result, "output:",predictions)

        init_angle = 2 * np.pi * result / 36
        self.obs_object = np.concatenate([[radius], self.model.body_pos[bid], obj_rotation6d, [direction]],
                                         axis=0)
        self.obs_object = self.obs_object.reshape((1, 11))

        self.obj_pos = random_pos
        self.obj_rotation = r.as_matrix()
        self.normal_vector = self.obj_rotation @ self.o_margin
        self.obj_normal = [0, 0, 0]
        for idx in range(3):
            self.obj_normal[idx] = self.normal_vector[idx][0] + self.obj_pos[idx]

        return r.as_matrix().tolist(), obj, radius, init_angle

    def save_frame_data(self, ee):
        r = R.from_euler('xyz', ee[1][3:6], degrees=False)
        rpyfromvalve_rot = r.inv() * R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)
        ee_align = R.from_euler('z', 45, degrees=True)
        rpyfromvalve = (ee_align * rpyfromvalve_rot).as_matrix()

        xyzfromvalve_rot = (R.from_matrix(self.obj_rotation) * R.from_matrix(self.T_vv)).as_matrix()
        xyzfromvalve_rot = np.concatenate([xyzfromvalve_rot, [[0, 0, 0]]], axis=0)
        xyzfromvalve_rot = np.concatenate(
            [xyzfromvalve_rot, [[self.obj_normal[0]], [self.obj_normal[1]], [self.obj_normal[2]], [1]]], axis=1)

        xyzfromvalve = inv(xyzfromvalve_rot) @ np.array([[ee[1][0]], [ee[1][1]], [ee[1][2]], [1]])

        if len(self.rpyfromvalve_data) == 0:
            self.rpyfromvalve_data = rpyfromvalve.reshape(1, 3, 3)
            self.xyzfromvalve_data = xyzfromvalve[0:3].reshape(1, 3)
            self.gripper_data = ee[2]
        else:
            self.rpyfromvalve_data = np.concatenate([self.rpyfromvalve_data, [rpyfromvalve]], axis=0)
            self.xyzfromvalve_data = np.concatenate([self.xyzfromvalve_data, [xyzfromvalve[0:3].reshape(3)]], axis=0)
            self.gripper_data = np.concatenate([self.gripper_data, ee[2]], axis=0)

    def read_file(self):
        with open(
                '/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dr_heuristic.txt',
                'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dr = list(map(float, f_list))
        with open(
                '/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dp_heuristic.txt',
                'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dp = list(map(float, f_list))

        with open(
                '/home/kist-robot2/catkin_ws/src/franka_emika_panda/build/devel/lib/franka_emika_panda/dy_heuristic.txt',
                'r') as f:
            f_line = f.readline()  # 파일 한 줄 읽어오기
            f_list = f_line.split()  # 그 줄을 list에 저장

            self.dy = list(map(float, f_list))

    def mujoco_xml(self):
        if self.rendering:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None

        del self.model
        del self.data
        s = randint(5, 9)
        # s=7
        handle_xml = f'''
            <mujocoinclude>
                <compiler angle="radian" meshdir="meshes/" autolimits="true"/>
                <size njmax="500" nconmax="100" />
                <visual>
                    <global offwidth="3024" offheight="1680" />
                    <quality shadowsize="4096" offsamples="8" />
                    <map force="0.1" fogend="5" />
                </visual>


                <asset>

                    <mesh name="handle_base" file="objects/handle_base.STL" scale="{s} {s} {s}"/>
                    <mesh name="handle_base0" file="objects/handle_base/handle_base000.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle_base1" file="objects/handle_base/handle_base001.obj" scale="{s} {s} {s}"/>

                    <mesh name="handle" file="objects/handle.STL" scale="{s} {s} {s}"/>


                    <mesh name="handle0" file="objects/handle2/handle000.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle1" file="objects/handle2/handle001.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle2" file="objects/handle2/handle002.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle3" file="objects/handle2/handle003.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle4" file="objects/handle2/handle004.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle5" file="objects/handle2/handle005.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle6" file="objects/handle2/handle006.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle7" file="objects/handle2/handle007.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle8" file="objects/handle2/handle008.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle9" file="objects/handle2/handle009.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle10" file="objects/handle2/handle010.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle11" file="objects/handle2/handle011.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle12" file="objects/handle2/handle012.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle13" file="objects/handle2/handle013.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle14" file="objects/handle2/handle014.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle15" file="objects/handle2/handle015.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle16" file="objects/handle2/handle016.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle17" file="objects/handle2/handle017.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle18" file="objects/handle2/handle018.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle19" file="objects/handle2/handle019.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle20" file="objects/handle2/handle020.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle21" file="objects/handle2/handle021.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle22" file="objects/handle2/handle022.obj" scale="{s} {s} {s}"/>
                    <mesh name="handle23" file="objects/handle2/handle023.obj" scale="{s} {s} {s}"/>
                </asset>

                <contact>
                    <exclude name="handle_contact" body1="handle_base" body2="handle_handle"/>
                </contact>

            </mujocoinclude>
        '''

        # Now you can write the XML content to a file
        with open('/home/kist-robot2/catkin_ws/src/franka_emika_panda/model/franka_emika_panda/assets_handle.xml',
                  'w') as file:
            file.write(handle_xml)
        return s