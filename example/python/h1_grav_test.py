import pinocchio as pin
import numpy as np
import math
import sys
import time

# from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
# from unitree_sdk2py.utils.crc import CRC

from robot_descriptions.loaders.pinocchio import load_robot_description

kNumMotors = 20

init_joint = np.array([
    0.0, -0.2, 0.5, 0.0, -0.2, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0
],
dtype=float) # 19+1


robot = load_robot_description("h1_mj_description")

model = robot.model
data = robot.data


q0 = np.array(
    [
        0, -0.2, 0.5, 0, -0.2, 0.5,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 
        0 ,0 ,0 ,0 ,0 ,0
    ]
)


v0 = np.zeros(model.nv)
a0 = np.zeros(model.nv)


g_grav = pin.rnea(model, data, q0, v0, a0)

print(g_grav)

breakpoint()
