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

from lowlevel_sdk.cyan_armwaisthead_cmd_lcmt import cyan_armwaisthead_cmd_lcmt
from lowlevel_sdk.cyan_armwaisthead_data_lcmt import cyan_armwaisthead_data_lcmt


'''
Example code for sending and receiving upperbody joint-level commands to/from the robot.
'''


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
    while True:
        for i in range(1000):
            start_time = time.time()
            msg = cyan_armwaisthead_cmd_lcmt()
            # left arm(0~5), right arm(6~11), waist(12), head(13~15)
            # shoulder -> hand
            msg.q_des = [0.0 for i in range(18)]  # desired motor position
            msg.qd_des = [0.0 for i in range(18)]  # desired motor velocity
            msg.kp_joint = [0.0 for i in range(18)]  # motor kp
            msg.kd_joint = [0.0 for i in range(18)]  # motor kd
            lcm_ins.publish("controller2robot_ahw", msg.encode())

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
    msg = cyan_armwaisthead_data_lcmt.decode(data)
    print(f"received q: {msg.q}\nqd: {msg.qd}\ntau: {msg.tauIq}\n")


def main():
    lcm_ins.subscribe("robot2controller_ahw", callback)
    threading.Thread(target=lcm_handle).start()
    controller2robot_example()


if __name__ == "__main__":
    main()
