import time
import sys
import numpy as np
import pinocchio as pin    
# from pinocchio import casadi as cpin                
# from pinocchio.robot_wrapper import RobotWrapper    
# from pinocchio.visualize import MeshcatVisualizer 

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

from robot_descriptions.loaders.pinocchio import load_robot_description

kNumMotors = 20

init_joint = np.array([
    0.0, -0.2, 0.5, 0.0, -0.2, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0
],
dtype=float) # 19

motor_cmd = np.array([
    0.0, -0.2, 0.5, 0.0, -0.2, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0
],
dtype=float) # 20

q = np.array([])
tau = np.array([])

#######################
class Robot_IK:
    def __init__(self):
        self.robot = load_robot_description("h1_mj_description")

        self.model = self.robot.model
        self.data = self.robot.data

        self.frame_id = self.model.getFrameId("right_ankle_link") 

        print("dof: ",self.model.nv)

        self.q0 = np.zeros(kNumMotors+6) # seems dof is 25, but coordinate is 26 !
        self.q0[7:] = init_joint
        self.q0[6] = 1
            
        # q_ref = pin.integrate(self.model, q0, 0.03* np.random.rand(self.model.nv))

        self.v0 = np.zeros(self.model.nv)
        self.a0 = np.zeros(self.model.nv)
    
        # self.data_sim = self.model.createData()
        # self.data_control = self.model.createData()

        # breakpoint()

        contact_models = []
        contact_datas = [] 

        frame = self.model.frames[self.frame_id]

        # get the id of the body link (pelvis)
        self.bl_id = self. model.getFrameId("pelvis")

        self.contact_model = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D, self.model, frame.parentJoint, frame.placement
        )

        contact_models.append(self.contact_model)
        contact_datas.append(self.contact_model.createData())


        self.num_constraints = 1
        self.contact_dim = 6 * self.num_constraints

        # pin.initConstraintDynamics(self.model, self.data_sim, contact_models)

        self.q = self.q0.copy()
        self.v = self.v0.copy()
        self.tau = np.zeros(self.model.nv - 6)

    def ik_func(self, q0):
        self.q0 = q0

        pin.framesForwardKinematics(self.model, self.data, self.q0)

        self.g_grav = pin.rnea(self.model, self.data, self.q0, self.v0, self.a0) # 25

        # print("grav. vec.: ",self.g_grav)

        g_bl = self.g_grav[:6]
        g_j = self.g_grav[6:]


        Js__foot_q = np.copy(pin.computeFrameJacobian(self.model, self.data, self.q0, self.frame_id, pin.LOCAL))

        # get the jacobian between contact foot and body linktau
        Js__foot_bl = np.copy(Js__foot_q[:6, :6]) 

        Jc__foot_bl_T = np.zeros([6, 6 * self.num_constraints])

        # transpot
        Jc__foot_bl_T[:, :] = np.vstack(Js__foot_bl).T


        # Now I only need to do the pinv to compute the contact forces
        ls = np.linalg.pinv(Jc__foot_bl_T) @ g_bl # This is (3)


        # Contact forces at local coordinates 
        # print("ls: ",ls)

        ###############

        # Contact forces at base link frame
        l_sp = pin.Force(ls)
        l_sp__bl = self.data.oMf[self.bl_id].actInv(self.data.oMf[self.frame_id].act(l_sp))

        Js_foot_j = np.copy(Js__foot_q[:6, 6:])
        Jc__foot_j_T = np.zeros([self.model.nv-6, 6 * self.num_constraints])
        Jc__foot_j_T[:, :] = np.vstack(Js_foot_j).T

        self.tau = g_j - Jc__foot_j_T @ ls

        # print("calc. torq.:", self.tau)

        return self.tau
#######################

dt = 0.002
runing_time = 0.0
crc = CRC()

class State: 
    def __init__(self):
        self.low_state = None  
        self.q = np.zeros(kNumMotors+6)
        self.q[6] = 1
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10)

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        # print(msg.motor_state[20].q)

        for i in range(9):
            self.q[i+7] = msg.motor_state[i].q
        for i in range(10):
            self.q[i+16] = msg.motor_state[i+10].q

        # body rpy
        
        # for i in range(3):
        #     self.q[i] = msg.imu_state.rpy[i]

        print("Joint state q:", self.q)
        print("RPY:", msg.imu_state.quaternion)
            

input("Press enter to start")

if __name__ == '__main__':
    h1_ik = Robot_IK()

    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    # sub = ChannelSubscriber("rt/lowstate", LowState_)
    # sub.Init(LowStateMessageHandler, 10)

    state = State()
    state.__init__()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(kNumMotors):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        # print("Joint state q:", state.q)
        tau = h1_ik.ik_func(state.q)
        # print("Grav. torque:", tau)
        
        # motor - joint transform
        for i in range(9):
            motor_cmd[i] = tau[i]
        for i in range(10):
            motor_cmd[i+10] = tau[i+9]


        # Total time for standing up or standing down is about 1.2s
        for i in range(kNumMotors):
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = motor_cmd[i]

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
