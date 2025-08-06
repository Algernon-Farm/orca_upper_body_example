# BSD 3-Clause License
# Copyright (c) 2025 Cyan Technologies Co., Ltd.
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import time
import threading

import lcm
import numpy as np

from lowlevel_sdk.cyan_armwaisthead_cmd_lcmt import cyan_armwaisthead_cmd_lcmt
from lowlevel_sdk.cyan_armwaisthead_data_lcmt import cyan_armwaisthead_data_lcmt
from t_curve_planner import interpolate_t_curve

"""
Example code for sending and receiving upperbody joint-level commands to/from the robot.
"""

cur_ahw_q = None

MOVE_BACK_TO_NOMINAL = False  # NOTE: set to true to move the robot arm back to nominal pose after setting a random pose

#
# localhost: lcm_ins = lcm.LCM()
# Multiple hosts: UDP Multicast Setup see https://lcm-proj.github.io/lcm/content/multicast-setup.html
lcm_ins = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")


def lcm_handle():
    try:
        while True:
            lcm_ins.handle()
    except KeyboardInterrupt:
        pass


def controller2robot_example():
    """send motor position commands to robot"""
    while cur_ahw_q is None:
        time.sleep(0.1)
        print("waiting for robot to be ready")

    target_pos = np.array(
        [
            -0.2,  # larm_joint6
            0.3,  # larm_joint5
            -1.0,  # larm_joint4
            0.4,  # larm_joint3
            0.8,  # larm_joint2
            0.1,  # larm_joint1
            -0.4,  # rarm_joint6
            -0.2,  # rarm_joint5
            1.0,  # rarm_joint4
            0.15,  # rarm_joint3
            0.4,  # rarm_joint2
            0.2,  # rarm_joint1
            -0.4,  # trunk_joint1
            0.15,  # head_roll_joint
            -0.3,  # head_pitch_joint
            0.15,  # head_yaw_joint
            0.0,  # not in use
            0.0,  # not in use
        ]
    )
    _, t_curv_q, t_curv_qd, _ = interpolate_t_curve(np.array(cur_ahw_q), target_pos, 18)
    for i in range(len(t_curv_q)):
        start_time = time.time()
        lcm_msg = cyan_armwaisthead_cmd_lcmt()
        # left arm(0~5), right arm(6~11), waist(12), head(13~15)
        # hand -> shoulder
        lcm_msg.q_des = t_curv_q[i]  # desired motor position
        lcm_msg.qd_des = t_curv_qd[i]  # desired motor velocity
        lcm_msg.qd_des[13:16] = [0.0, 0.0, 0.0]
        lcm_msg.kp_joint = [50.0] * 18  # motor kp
        lcm_msg.kp_joint[13:16] = [10.0, 10.0, 10.0]  # head kp
        lcm_msg.kd_joint = [1.0] * 18  # motor kd
        lcm_msg.kd_joint[13:16] = [0.2, 0.2, 0.2]  # head kd
        lcm_ins.publish("controller2robot_ahw", lcm_msg.encode())

        # make it a 1000hz loop
        # NOTE: this is not the best way to do this as we are using python
        end_time = time.time()

        if (end_time - start_time) < 0.001:
            time.sleep(0.001 - (end_time - start_time))

    # NOTE: to move the arm back to the nominal pose
    if MOVE_BACK_TO_NOMINAL:
        _, t_curv_q, t_curv_qd, _ = interpolate_t_curve(target_pos, np.zeros(18), 18)
        for i in range(len(t_curv_q)):
            start_time = time.time()
            lcm_msg = cyan_armwaisthead_cmd_lcmt()
            # left arm(0~5), right arm(6~11), waist(12), head(13~15)
            # shoulder -> hand
            lcm_msg.q_des = t_curv_q[i]  # desired motor position
            lcm_msg.qd_des = t_curv_qd[i]  # desired motor velocity
            lcm_msg.qd_des[13:16] = [0.0, 0.0, 0.0]
            lcm_msg.kp_joint = [50.0] * 18  # motor kp
            lcm_msg.kp_joint[13:16] = [10.0, 10.0, 10.0]  # head kp
            lcm_msg.kd_joint = [1.0] * 18  # motor kd
            lcm_msg.kd_joint[13:16] = [0.2, 0.2, 0.2]  # head kd
            lcm_ins.publish("controller2robot_ahw", lcm_msg.encode())

            # make it a 1000hz loop
            # NOTE: this is not the best way to do this as we are using python
            end_time = time.time()

            if (end_time - start_time) < 0.001:
                time.sleep(0.001 - (end_time - start_time))


def callback(channel: str, data: cyan_armwaisthead_data_lcmt):
    """
    subscribe motor position data from robot
    :param channel: robot2controller_ahw
    :param data: cyan_armwaisthead_data_lcmt
    """
    global cur_ahw_q
    msg = cyan_armwaisthead_data_lcmt.decode(data)
    cur_ahw_q = msg.q
    # print(f"received q: {msg.q}\nqd: {msg.qd}\ntau: {msg.tauIq}\n")


def main():
    lcm_ins.subscribe("robot2controller_ahw", callback)
    threading.Thread(target=lcm_handle).start()
    controller2robot_example()


if __name__ == "__main__":
    main()
